#include <ranges>

#include <SofaDiff/optimizers/GridSearchOptimizationLoop.h>
#include <SofaDiff/LossState.h>

#include <sofa/core/ObjectFactory.h>

#include "sofa/linearalgebra/CompressedRowSparseMatrixConstraintEigenUtils.h"


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
    for (const auto & parameter : l_parameters)
    {
        const auto resolution = static_cast<int>(parameter->getHyperparameter("resolution"));
        const auto size = parameter->getValueVector().size();
        for (unsigned long i = 0; i < size; i++)
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
    auto global_index = iteration;
    for (const auto & parameter : std::ranges::reverse_view(l_parameters))
    {
        const auto lowerBound = parameter->getHyperparameter("lowerBound");
        const auto upperBound = parameter->getHyperparameter("upperBound");
        const auto resolution = static_cast<int>(parameter->getHyperparameter("resolution"));
        const auto size = parameter->getValueVector().size();

        type::vector<SReal> nextValue(size);
        for (unsigned long i = size; i > 0; i--)
        {
            const auto parameter_index = global_index % resolution;
            nextValue[i-1] = lowerBound + parameter_index * (upperBound - lowerBound) / (resolution - 1);
            global_index = (global_index - parameter_index) / resolution;
        }

        parameter->setNextValueVector(nextValue);
    }
}

}
