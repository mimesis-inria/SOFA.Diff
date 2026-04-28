#include <SofaDiff/adjoints/StaticAdjointSolver.h>
#include <SofaDiff/DifferentiableAnimationLoop.h>
#include <SofaDiff/visitors/MechanicalAccumulateVecDeriv.h>
#include <SofaDiff/visitors/MechanicalPropagateVecDeriv.h>

#include <sofa/core/MechanicalParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalResetForceVisitor.h>


namespace sofadiff
{
using namespace sofa::simulation::mechanicalvisitor;
using namespace sofa::core;


void registerStaticAdjointSolver(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Adjoint solver for static problems.").add< StaticAdjointSolver >());
}

void StaticAdjointSolver::init()
{
    AdjointSolver::init();
    LinearSolverAccessor::init();

    m_positionGradientId = newVecId("gradient of the global loss wrt x");
    m_forceGradientId = newVecId("gradient of the global loss wrt f");
}

void StaticAdjointSolver::resetGradients(const ExecParams * params)
{
    auto *ctx = this->getContext()->getRootContext(); // TODO: maybe not the root context? But needs access to loss and parameters...
    const MechanicalParams mparams(*params);

    MechanicalResetForceVisitor(&mparams, m_positionGradientId).execute(ctx, false);
    resetParametersGradient();
}


void StaticAdjointSolver::solve(const ExecParams* params, SReal /*dt*/, MultiVecCoordId /*xResult*/, MultiVecDerivId /*vResult*/)
{
    auto *ctx = this->getContext()->getRootContext(); // TODO: maybe not the root context? But needs access to loss and parameters...
    const MechanicalParams mparams(*params);

    MechanicalAccumulateVecDeriv(&mparams, m_positionGradientId).execute(ctx, false);
    solveForPhysicalGradient();
    MechanicalPropagateVecDeriv(&mparams, m_forceGradientId).execute(ctx, false);
    propagateGradientsThroughForceFields(&mparams, m_forceGradientId);
}

void StaticAdjointSolver::solveForPhysicalGradient()
{
    SCOPED_TIMER("StaticAdjointSolver::solveForPhysicalGradient");
    auto *linearSolver = l_linearSolver.get();

    const auto * params = execparams::defaultInstance();
    simulation::common::MechanicalOperations mop(params, this->getContext());
    mop->setImplicit(true);
    static constexpr MatricesFactors::M m(0);
    static constexpr MatricesFactors::B b(0);
    static constexpr MatricesFactors::K k(-1);
    mop.setSystemMBKMatrix(m, b, k, linearSolver);

    linearSolver->getLinearSystem()->setSystemSolution(m_forceGradientId);
    linearSolver->getLinearSystem()->setRHS(m_positionGradientId);
    linearSolver->solveSystem();  // Solve -df/dx * lambda = dy/dx
    linearSolver->getLinearSystem()->dispatchSystemSolution(m_forceGradientId);

    mop.projectResponse(m_forceGradientId);  // Take the projective constraints into account
}

}