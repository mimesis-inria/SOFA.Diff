#include <SofaDiff/adjoints/AdjointSolver.h>

#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/VectorOperations.h>


namespace sofadiff
{

using namespace sofa::core;
using namespace sofa::core::objectmodel;

void AdjointSolver::init()
{
    const auto* ctx = this->getContext();
    ctx->get<BaseParameter> (&m_trainableParameters, BaseContext::SearchRoot);
    ctx->get<Parameterized> (&m_parameterizedForceFields, BaseContext::SearchRoot);
    ctx->get<LossState> (&m_lossStates, BaseContext::SearchRoot);
}

void AdjointSolver::bwdInit()
{
    // Tell the lossStates that they should return their d_gradient when asked for the given VecId
    for (const auto loss : m_lossStates)
        loss->setGradientVecId(getLossGradientId());
}


MultiVecDerivId AdjointSolver::newVecId(const char * name)
{
    simulation::common::VectorOperations vop(mechanicalparams::defaultInstance(), this->getContext()->getRootContext());
    const auto vecId = TMultiVecId<VecType::V_DERIV, VecAccess::V_WRITE>();
    behavior::MultiVecDeriv vec(&vop, vecId);
    vec.realloc(&vop, false, true, VecIdProperties{name, this->getClassName()});
    return vec.id();
}

void AdjointSolver::resetParametersGradient() const
{
    SCOPED_TIMER("AdjointSolver::resetParametersGradient");
    for (auto & parameter : m_trainableParameters)
        parameter->setGradient(0);
}

void AdjointSolver::propagateGradientsThroughForceFields(const MechanicalParams * mparams, const MultiVecDerivId & forceGradientId) const
{
    SCOPED_TIMER("AdjointSolver::propagateGradientsThroughForceFields");
    for (const auto &forceField: m_parameterizedForceFields)
        forceField->applyParametersJacobianTranspose(mparams, forceGradientId);
}

}
