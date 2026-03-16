#include <ranges>

#include <SofaDiff/optimizers/GridSearchOptimizationLoop.h>
#include <SofaDiff/LossState.h>

#include <sofa/core/ObjectFactory.h>


namespace sofadiff
{
using namespace sofa::core;

void registerGridSearchOptimizationLoop(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Grid search algorithm for optimization.").add< GridSearchOptimizationLoop >());
}

void GridSearchOptimizationLoop::init()
{
    OptimizationLoop::init();

    m_bestIteration = 0;
    m_lowestLossValue = std::numeric_limits<SReal>::max();
    m_gridSize = 1;
    std::vector<BaseParameter*> parameters;
    this->getContext()->get<BaseParameter>(&parameters, objectmodel::BaseContext::SearchDown);
    for (const auto parameter : parameters)
    {
        const auto resolution = static_cast<int>(parameter->getHyperparameter("resolution"));
        m_gridSize *= resolution;
    }

    if (d_maxOptimizationSteps.getValue() == 0)
    {
        d_maxOptimizationSteps.setValue(m_gridSize + 1);
    }

    setParameters(0);
    m_readyToUpdateParameters = true;
}

void GridSearchOptimizationLoop::resetOptimization()
{
    OptimizationLoop::resetOptimization();

    m_bestIteration = 0;
    m_lowestLossValue = std::numeric_limits<SReal>::max();
    // Update grid size?

    setParameters(0);
    m_readyToUpdateParameters = true;
}

void GridSearchOptimizationLoop::computeParametersNextValue(const ExecParams *params, const SReal dt)
{
    SOFA_UNUSED(params);
    SOFA_UNUSED(dt);

    const int currentStep = this->getCurrentOptimizationStep();

    SReal loss = 0;
    std::vector<LossState*> losses;
    this->getContext()->get<LossState>(&losses, objectmodel::BaseContext::SearchDown);
    for (const auto lossState : losses)
        loss += lossState->d_value.getValue()[0][0];

    if (loss < m_lowestLossValue)
    {
        m_lowestLossValue = loss;
        m_bestIteration = currentStep;
    }

    const int iteration = currentStep + 1 < m_gridSize ? currentStep + 1 : m_bestIteration;
    this->setParameters(iteration);
}

void GridSearchOptimizationLoop::setParameters(const int iteration)
{

    std::vector<BaseParameter*> parameters;
    this->getContext()->get<BaseParameter>(&parameters, objectmodel::BaseContext::SearchDown);
    auto global_index = iteration;
    for (const auto & parameter : std::ranges::reverse_view(parameters))
    {
        const auto lowerBound = parameter->getHyperparameter("lowerBound");
        const auto upperBound = parameter->getHyperparameter("upperBound");
        const auto resolution = static_cast<int>(parameter->getHyperparameter("resolution"));

        const auto parameter_index = global_index % resolution;
        parameter->setNextValueVector({lowerBound + parameter_index * (upperBound - lowerBound) / (resolution - 1)});
        global_index = (global_index - parameter_index) / resolution;
    }
}

}