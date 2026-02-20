#include <SofaDiff/adjoints/ImplicitAdjointSolver.h>
#include <SofaDiff/DifferentiableAnimationLoop.h>

#include <sofa/core/MechanicalParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/MechanicalOperations.h>
// #include <sofa/simulation/mechanicalvisitor/MechanicalResetForceVisitor.h>
#include <sofa/simulation/VectorOperations.h>

#include "SofaDiff/visitors/MechanicalAccumulateVecDeriv.h"


namespace sofadiff
{
using namespace sofa::core;


void registerImplicitAdjointSolver(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Adjoint solver for implicit Euler.").add< ImplicitAdjointSolver >());
}

void ImplicitAdjointSolver::init()
{
    auto* ctx = this->getContext();
    auto* params = mechanicalparams::defaultInstance();
    simulation::common::VectorOperations vop(params, ctx);

    m_positionGradientId = TMultiVecId<VecType::V_DERIV, VecAccess::V_WRITE>();
    behavior::MultiVecDeriv positionGradient(&vop, m_positionGradientId);
    positionGradient.realloc(&vop, false, true, VecIdProperties{"gradient of the loss wrt x", this->getClassName()});
    m_positionGradientId = positionGradient.id();

    m_velocityGradientId = TMultiVecId<VecType::V_DERIV, VecAccess::V_WRITE>();
    behavior::MultiVecDeriv velocityGradient(&vop, m_velocityGradientId);
    velocityGradient.realloc(&vop, false, true, VecIdProperties{"gradient of the loss wrt v", this->getClassName()});
    m_velocityGradientId = velocityGradient.id();

    m_deltaVelocityGradientId = TMultiVecId<VecType::V_DERIV, VecAccess::V_WRITE>();
    behavior::MultiVecDeriv deltaVelocityGradient(&vop, m_deltaVelocityGradientId);
    deltaVelocityGradient.realloc(&vop, false, true, VecIdProperties{"gradient of the loss wrt dv", this->getClassName()});
    m_deltaVelocityGradientId = deltaVelocityGradient.id();
}

void ImplicitAdjointSolver::solve(const ExecParams * params, SReal dt, MultiVecCoordId, MultiVecDerivId)
{
    // set LossState gradient to the derivative of the total loss wrt the current instant loss
    // yn.grad = dy/dyn

    // propagate said gradient to the main dofs (position & velocity) through the mappings
    // this gradient is added to the main dofs gradients, as it should be
    // xn.grad += yn.grad * dyn/dxn
    // vn.grad += yn.grad * dyn/dvn
    // ?? pn.grad += yn.grad * dyn/dpn ??
    // MechanicalAccumulateVecDeriv(&mparams, ) // TODO: add constructor with two VecIds (in & out)

    // solve the system for the "force gradient"
    // F(dv; xn, vn, pn) = 0 --> f.grad * dF/d(dv) = - dv.grad = - vn.grad - dt * xn.grad
    // solveForForceGradient(params, dt);

    // propagate the force gradient to the mapped states through the mappings

    // propagate the "force gradient" to the parameters through the ParameterizedForceFields
    // ?? pn.grad += f.grad * df/dp

    // update the "dofs gradient" by propagating the "force gradient"
    // this gradient replaces the previous one
    // xn.grad = f.grad * df/dx
    // vn.grad = f.grad * df/dv
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
    // const MatricesFactors::M mFact (1 + tr * h * d_rayleighMass.getValue());
    // const MatricesFactors::B bFact (-tr * h);
    // const MatricesFactors::K kFact (-tr * h * (tr * h + d_rayleighStiffness.getValue()));
    // mop.setSystemMBKMatrix(mFact, bFact, kFact, l_linearSolver.get());

    // Solve
    linearSolver->getLinearSystem()->setSystemSolution(s_physicalGradientId);
    linearSolver->getLinearSystem()->setRHS(m_deltaVelocityGradientId);
    linearSolver->solveSystem();  // Solve -dF/d(dv) * dy/df = dy/dx
    linearSolver->getLinearSystem()->dispatchSystemSolution(s_physicalGradientId);
}


}
