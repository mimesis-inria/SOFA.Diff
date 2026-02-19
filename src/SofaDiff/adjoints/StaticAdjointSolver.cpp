#include <SofaDiff/adjoints/StaticAdjointSolver.h>
#include <SofaDiff/DifferentiableAnimationLoop.h>
#include <SofaDiff/visitors/MechanicalAccumulateVecDeriv.h>
#include <SofaDiff/visitors/MechanicalPropagateVecDeriv.h>
#include <SofaDiff/ParameterizedForceField.h>
#include <SofaDiff/LossState.h>

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
    LinearSolverAccessor::init();
    const auto* ctx = this->getContext();

    ctx->get<BaseParameter> (&m_trainableParameters, BaseContext::SearchRoot);
    ctx->get<Parameterized> (&m_parameterizedForceFields, BaseContext::SearchRoot);
    ctx->get<LossState> (&m_lossStates, BaseContext::SearchRoot);

    initializeLossGradientToOne();
}

void StaticAdjointSolver::solve(const ExecParams* params, SReal /*dt*/, MultiVecCoordId /*xResult*/, MultiVecDerivId /*vResult*/)
{
    auto *ctx = this->getContext()->getRootContext(); // TODO: maybe not the root context? But needs access to loss and parameters...
    const MechanicalParams mparams(*params);

    MechanicalResetForceVisitor(&mparams, s_geometricGradientId).execute(ctx, false);
    resetParametersGradient();
    MechanicalAccumulateVecDeriv(&mparams, s_geometricGradientId).execute(ctx, false);
    solveForPhysicalGradient();
    MechanicalPropagateVecDeriv(&mparams, s_physicalGradientId).execute(ctx, false);
    {
        SCOPED_TIMER("StaticAdjointSolver::applyParametersJacobianTranspose");
        for (const auto &forceField: m_parameterizedForceFields)
            forceField->applyParametersJacobianTranspose(&mparams, s_physicalGradientId);
    }
}

void StaticAdjointSolver::initializeLossGradientToOne()
{
    SCOPED_TIMER("StaticAdjointSolver::initializeLossGradientToOne");
    for (const auto loss : m_lossStates)
    {
        if (loss == nullptr)
        {
            msg_error() << "Bad link to the loss object";
            this->d_componentState.setValue(ComponentState::Invalid);
            return;
        }
        const auto& gradient = s_geometricGradientId.getId(loss);
        // TODO: handle case where we cannot write in gradient
        helper::WriteAccessor<Data<VecDeriv_t<defaulttype::Vec1Types>> > lossGradient = loss->write(gradient);
        lossGradient[0] = sofa::Deriv_t<defaulttype::Vec1Types> (1);
    }
}


void StaticAdjointSolver::resetParametersGradient() const
{
    SCOPED_TIMER("StaticAdjointSolver::resetParametersGradient");
    for (auto & parameter : m_trainableParameters)
        parameter->resetGradient();
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

    linearSolver->getLinearSystem()->setSystemSolution(s_physicalGradientId);
    linearSolver->getLinearSystem()->setRHS(s_geometricGradientId);
    linearSolver->solveSystem();  // Solve -df/dx * lambda = dy/dx
    linearSolver->getLinearSystem()->dispatchSystemSolution(s_physicalGradientId);
}

}