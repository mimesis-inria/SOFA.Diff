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
        d_maxOptimizationSteps.setValue(m_gridSize);
    }

    setParameters(0);
    m_readyToUpdateParameters = true;
}

void GridSearchOptimizationLoop::resetOptimization()
{
    OptimizationLoop::resetOptimization();

    // Update grid size?

    setParameters(0);
    m_readyToUpdateParameters = true;
}

void GridSearchOptimizationLoop::setParametersNextValue()
{
    const int currentStep = this->getCurrentOptimizationStep();
    if (currentStep + 1 < m_gridSize)
        this->setParameters(currentStep + 1);
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