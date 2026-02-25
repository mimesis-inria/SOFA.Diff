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
    AdjointSolver::init();
    LinearSolverAccessor::init();

    auto* ctx = this->getContext();
    auto* params = mechanicalparams::defaultInstance();
    simulation::common::VectorOperations vop(params, ctx);

    m_positionGradientId = newVecId("gradient of the loss wrt x");
    m_velocityGradientId = newVecId("gradient of the loss wrt v");
    m_deltaVelocityGradientId = newVecId("gradient of the loss wrt dv");
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

void ImplicitAdjointSolver::solve(const ExecParams * params, SReal dt, MultiVecCoordId, MultiVecDerivId)
{
    // set LossState gradient to the derivative of the total loss wrt the current instant loss
    // yn.grad = dy/dyn
    // No: done in DifferentiableAnimationLoop::stepAdjoint() that knows the context

    auto *ctx = this->getContext()->getRootContext();
    const auto mparams = MechanicalParams(*params);

    // propagate said gradient to the main dofs (position & velocity) through the mappings
    // this gradient is added to the main dofs gradients, as it should be
    // xn.grad += yn.grad * dyn/dxn  <--- Only this one for now (because standard mapping are functions of x only)
    // vn.grad += yn.grad * dyn/dvn
    // ?? pn.grad += yn.grad * dyn/dpn ??
    simulation::mechanicalvisitor::MechanicalResetForceVisitor(&mparams, s_geometricGradientId).execute(ctx, false);
    MechanicalAccumulateVecDeriv(&mparams, s_geometricGradientId).execute(ctx, false);

    auto * local_ctx = this->getContext();
    simulation::common::VectorOperations vop(&mparams, local_ctx);
    vop.v_peq(m_positionGradientId, s_geometricGradientId);

    // solve the system for the "force gradient"
    // F(dv; xn, vn, pn) = 0 --> f.grad * dF/d(dv) = - dv.grad = - vn.grad - dt * xn.grad
    solveForForceGradient(params, dt);

    // propagate the force gradient to the mapped states through the mappings
    MechanicalPropagateVecDeriv(&mparams, s_physicalGradientId).execute(ctx, false);

    // propagate the "force gradient" to the parameters through the ParameterizedForceFields
    // ?? pn.grad += f.grad * df/dp
    propagateGradientsThroughForceFields(&mparams, s_physicalGradientId);

    // update the "dofs gradient" by propagating the "force gradient"
    // this gradient replaces the previous one
    // xn.grad = f.grad * df/dx
    // vn.grad = f.grad * df/dv
    updateVelocityGradient(mparams, dt);
    updatePositionGradient(mparams, dt);
    // In this order because velocity gradient depends on position gradient (before update)
}

void ImplicitAdjointSolver::solveForForceGradient(const ExecParams *params, SReal dt)
{
    SCOPED_TIMER("ImplicitAdjointSolver::solveForPhysicalGradient");
    auto* ctx = this->getContext();

    // dv.grad = vn.grad + dt * xn.grad
    const auto mparams = MechanicalParams(*params);
    simulation::common::VectorOperations vop(&mparams, ctx);
    vop.v_eq(m_deltaVelocityGradientId, m_velocityGradientId);
    vop.v_peq(m_deltaVelocityGradientId, m_positionGradientId, dt);

    // dF/d(dv) = M - dt^2*K - dt*B
    auto *linearSolver = l_linearSolver.get();
    simulation::common::MechanicalOperations mop(params, ctx);
    mop->setImplicit(true);
    const MatricesFactors::M mFact (1.0); // (1 + tr * dt * d_rayleighMass.getValue());
    const MatricesFactors::B bFact (-dt); // (-tr * dt);
    const MatricesFactors::K kFact (-dt*dt); // (-tr * dt * (tr * dt + d_rayleighStiffness.getValue()));
    mop.setSystemMBKMatrix(mFact, bFact, kFact, l_linearSolver.get());

    // Solve
    linearSolver->getLinearSystem()->setSystemSolution(s_physicalGradientId);
    linearSolver->getLinearSystem()->setRHS(m_deltaVelocityGradientId);
    linearSolver->solveSystem();  // Solve dF/d(dv) * dy/df = dy/dx (note the absence of the minus sign, cf below)
    linearSolver->getLinearSystem()->dispatchSystemSolution(s_physicalGradientId);

    // Multiply by dt to get the force gradient (since F = M*dv - dt*f ==> f.grad = -dt * F.grad)
    // The minus sign cancels out with the minus sign that was omitted in the system above
    vop.v_teq(s_physicalGradientId, dt);
}

void ImplicitAdjointSolver::updatePositionGradient(MechanicalParams mparams, SReal dt)
{
    auto *ctx = this->getContext();

    const ConstMultiVecDerivId dx = mparams.dx();
    mparams.setDx(s_physicalGradientId);
    // setDf(df);
    mparams.setBFactor(0.0);
    mparams.setKFactor(1.0);
    mparams.setMFactor(0.0);
    simulation::mechanicalvisitor::MechanicalAddMBKdxVisitor(&mparams, m_positionGradientId, false).execute(ctx);
    mparams.setDx(dx);
}


void ImplicitAdjointSolver::updateVelocityGradient(MechanicalParams mparams, SReal dt)
{
    auto *ctx = this->getContext();
    simulation::common::VectorOperations vop(&mparams, ctx);
    vop.v_peq(m_velocityGradientId, m_positionGradientId, dt);

    const ConstMultiVecDerivId dx = mparams.dx();
    mparams.setDx(s_physicalGradientId);
    // setDf(df);
    mparams.setBFactor(1.0);
    mparams.setKFactor(dt);
    mparams.setMFactor(0.0);
    simulation::mechanicalvisitor::MechanicalAddMBKdxVisitor(&mparams, m_velocityGradientId, false).execute(ctx);
    mparams.setDx(dx);
}

}
