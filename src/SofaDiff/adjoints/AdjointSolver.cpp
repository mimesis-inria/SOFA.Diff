#include <SofaDiff/adjoints/AdjointSolver.h>

#include <sofa/helper/ScopedAdvancedTimer.h>


namespace sofadiff
{

using namespace sofa::core;

void AdjointSolver::init()
{
    const auto* ctx = this->getContext();
    ctx->get<BaseParameter> (&m_trainableParameters, objectmodel::BaseContext::SearchRoot);
    ctx->get<Parameterized> (&m_parameterizedForceFields, objectmodel::BaseContext::SearchRoot);
}

void AdjointSolver::resetParametersGradient() const
{
    SCOPED_TIMER("AdjointSolver::resetParametersGradient");
    for (auto & parameter : m_trainableParameters)
        parameter->resetGradient();
}

}
