#include <SofaDiff/adjoints/ImplicitAdjointSolver.h>
#include <SofaDiff/DifferentiableAnimationLoop.h>
#include <SofaDiff/visitors/MechanicalAccumulateVecDeriv.h>
#include <SofaDiff/visitors/MechanicalPropagateVecDeriv.h>

#include <sofa/core/MechanicalParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalResetForceVisitor.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalAddMBKdxVisitor.h>
#include <sofa/simulation/VectorOperations.h>


namespace sofadiff
{
using namespace sofa::core;


void registerImplicitAdjointSolver(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Adjoint solver for implicit Euler.").add< ImplicitAdjointSolver >());
}

void ImplicitAdjointSolver::init()
{
    LinearSolverAccessor::init();
    AdjointSolver::init();

    m_lossGradientId = newVecId("gradient of the instant loss wrt x");
    m_positionGradientId = newVecId("gradient of the global loss wrt x");
    m_velocityGradientId = newVecId("gradient of the global loss wrt v");
    m_deltaVelocityGradientId = newVecId("gradient of the global loss wrt dv");
    m_residualGradientId = newVecId("gradient of the global loss wrt the residual F");
    m_forceGradientId = newVecId("gradient of the global loss wrt f");
}

void ImplicitAdjointSolver::resetGradients(const ExecParams * params)
{
    auto *ctx = this->getContext()->getRootContext();
    const auto mparams = MechanicalParams(*params);
    simulation::mechanicalvisitor::MechanicalResetForceVisitor(&mparams, m_positionGradientId).execute(ctx, false);
    simulation::mechanicalvisitor::MechanicalResetForceVisitor(&mparams, m_velocityGradientId).execute(ctx, false);
    simulation::mechanicalvisitor::MechanicalResetForceVisitor(&mparams, m_deltaVelocityGradientId).execute(ctx, false);
    resetParametersGradient();
}

void ImplicitAdjointSolver::solve(const ExecParams * params, const SReal dt, MultiVecCoordId, MultiVecDerivId)
{
    auto * localContext = this->getContext();
    auto * rootContext = localContext->getRootContext();
    const auto mparams = MechanicalParams(*params);
    simulation::common::VectorOperations vop(&mparams, localContext);

    // Notations:
    // * This method differentiates the time step between instants n and n+1
    // * qn and qnp1 denote the quantity q at instant n and at instant n+1, respectively
    // * q.grad denotes the gradient of the total loss y wrt to the quantity q: q.grad = dy/dq

    // Assume we are given:
    // * xnp1.grad, vnp1.grad, pnp1.grad: the gradients of the total loss y wrt xnp1, vnp1 and pnp1 respectively,
    //   without their contribution to y through the instant loss ynp1
    // * ynp1.grad: the derivative of the total loss y wrt the instant loss ynp1

    // Propagate ynp1.grad to the main dofs (position & velocity) and the parameters through the mappings, to get
    // the contribution of the current instant loss on xnp1.grad, vnp1.grad and pnp1.grad:
    // xnp1.contrib = ynp1.grad * d(ynp1)/d(xnp1)  <--- Only this one for now (because standard mappings are functions of x only)
    // vnp1.contrib = ynp1.grad * d(ynp1)/d(vnp1)
    // pnp1.contrib = ynp1.grad * d(ynp1)/d(pnp1)
    simulation::mechanicalvisitor::MechanicalResetForceVisitor(&mparams, m_lossGradientId).execute(rootContext, false);
    MechanicalAccumulateVecDeriv(&mparams, m_lossGradientId).execute(rootContext, false);

    // Add this contribution to the gradients computed during the previous stepAdjoint():
    // xnp1.grad += xnp1.contrib  <--- Only this one for now (because standard mapping are functions of x only)
    // vnp1.grad += vnp1.contrib
    // pnp1.grad += pnp1.contrib (assuming the parameters are not time-dependent)
    vop.v_peq(m_positionGradientId, m_lossGradientId);

    // Compute the gradient of the increment in velocity dv through the position and velocity:
    // Since vnp1 = vn + dv  and  xnp1 = xn + dt*(vn + dv), we have:
    // dv.grad = dy/d(vnp1) * d(vnp1)/d(dv)  +  dy/d(xnp1) * d(xnp1)/d(dv)
    //         =  vnp1.grad                  +   xnp1.grad * dt
    vop.v_eq(m_deltaVelocityGradientId, m_velocityGradientId);
    vop.v_peq(m_deltaVelocityGradientId, m_positionGradientId, dt);

    // Consider that the increment in velocity is an implicit function of xn, vn, pn,
    // denoted by dv(xn, vn, pn), that is the solution of a system F(dv; xn, vn, pn) = 0.
    // Therefore, backpropagate the gradients through the ODE solver with "implicit differentiation".

    // First solve the system for the "residual gradient" F.grad and propagate it through the mappings:
    // F(dv; xn, vn, pn) = 0   ==>   F.grad * dF/d(dv) = -dv.grad
    // (Note that dy/dF does not exist here since F is a function, but the notation F.grad is convenient anyway.)
    solveForResidualGradient(params, dt);
    MechanicalPropagateVecDeriv(&mparams, m_residualGradientId).execute(rootContext, false);

    // Then use this "residual gradient" to update the gradients of the dofs and the parameters, using the relations:
    // vnp1 = vn + dv(xn, vn, pn)
    // xnp1 = xn + dt * (vn + dv(xn, vn, pn))
    // F(dv; xn, vn, pn) = M * dv  -  dt * f(xn + dt*(vn + dv), vn + dv, pn)

    // vn.grad = vnp1.grad * d(vnp1)/d(vn)      +  xnp1.grad * d(xnp1)/d(vn)
    //         = vnp1.grad * (I + d(dv)/d(vn))  +  xnp1.grad * (dt*I + dt*d(dv)/d(vn))
    //         = vnp1.grad  +  dt * xnp1.grad  +  (dv.grad * d(dv)/d(vn))
    //         = vnp1.grad  +  dt * xnp1.grad  +  F.grad * dF/dvn
    // with dF/dvn = - dt^2 * df/dx  - dt * df/dv
    vop.v_peq(m_velocityGradientId, m_positionGradientId, dt);
    addMatrixVectorProduct(m_velocityGradientId, m_residualGradientId, -dt*dt, -dt);

    // xn.grad = vnp1.grad * d(vnp1)/d(xn)  +  xnp1.grad * d(xnp1)/d(xn)
    //         = vnp1.grad * d(dv)/d(xn)    +  xnp1.grad * (I + dt*d(dv)/d(xn))
    //         = xnp1.grad  +  (dv.grad * d(dv)/d(xn))
    //         = xnp1.grad  +  F.grad * dF/dxn
    // with dF/dxn = - dt * df/dx
    addMatrixVectorProduct(m_positionGradientId, m_residualGradientId, -dt, 0.0);

    // pn.grad = vnp1.grad * d(vnp1)/d(pn)  +  xnp1.grad * d(xnp1)/d(pn)
    //         = vnp1.grad * d(dv)/d(pn)    +  xnp1.grad * dt*d(dv)/d(pn)
    //         = dv.grad * d(dv)/d(pn)
    //         = F.grad * dF/dpn
    // with dF/dpn = - dt * df/dp
    vop.v_eq(m_forceGradientId, m_residualGradientId);
    vop.v_teq(m_forceGradientId, -dt);
    propagateGradientsThroughForceFields(&mparams, m_forceGradientId);
}

void ImplicitAdjointSolver::solveForResidualGradient(const ExecParams *params, const SReal dt)
{
    SCOPED_TIMER("ImplicitAdjointSolver::solveForForceGradient");

    // Assemble the matrix of the system.
    // Since F(dv; xn, vn, pn) = M * dv  -  dt * f(xn + dt*(vn + dv), vn + dv, pn), we have
    // dF/d(dv) = M - dt^2*K - dt*B

    // We could actually use the same factors as the EulerImplicitSolver, assuming firstOrder==false (because we need deltaVelocity to be the unknown)
    // A more generic way to do things may be to check for the method of the EulerImplicitSolver and be consistent with it
    // or to do what we want to do independently of EulerImplicitSolver and use the code below (once the PR making it work is accepted)
    const MatricesFactors::M mFact (1.0); // (1 + tr * dt * d_rayleighMass.getValue());
    const MatricesFactors::B bFact (-dt); // (-tr * dt);
    const MatricesFactors::K kFact (-dt*dt); // (-tr * dt * (tr * dt + d_rayleighStiffness.getValue()));
    auto *linearSolver = l_linearSolver.get();
    simulation::common::MechanicalOperations mop(params, this->getContext());
    mop->setImplicit(true);
    mop.setSystemMBKMatrix(mFact, bFact, kFact, l_linearSolver.get());

    // Solve for -F.grad, assuming the symmetry of the matrix
    linearSolver->getLinearSystem()->setSystemSolution(m_residualGradientId);
    linearSolver->getLinearSystem()->setRHS(m_deltaVelocityGradientId);
    linearSolver->solveSystem();
    linearSolver->getLinearSystem()->dispatchSystemSolution(m_residualGradientId);

    // Multiply by -1 to get F.grad
    const auto mparams = MechanicalParams(*params);
    simulation::common::VectorOperations vop(&mparams, this->getContext());
    vop.v_teq(m_residualGradientId, -1.0);
}

void ImplicitAdjointSolver::addMatrixVectorProduct(const MultiVecDerivId& outVectorId, const MultiVecDerivId &inVectorId, const SReal kFact, const SReal bFact)
{
    auto mparams = *MechanicalParams::defaultInstance();
    mparams.setDx(inVectorId);
    mparams.setBFactor(bFact);
    mparams.setKFactor(kFact);
    mparams.setMFactor(0.0);
    simulation::mechanicalvisitor::MechanicalAddMBKdxVisitor(&mparams, outVectorId, false).execute(this->getContext());
}

}
