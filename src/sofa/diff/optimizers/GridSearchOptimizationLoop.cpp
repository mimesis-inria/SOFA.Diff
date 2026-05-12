/******************************************************************************
*                                 SOFA.Diff                                   *
*                              (c) 2026 INRIA                                 *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: Léo Bois, Paul Baksic                                              *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <ranges>

#include <sofa/diff/optimizers/GridSearchOptimizationLoop.h>
#include <sofa/diff/LossState.h>

#include <sofa/core/ObjectFactory.h>


namespace sofadiff
{
using namespace sofa::core;

void registerGridSearchOptimizationLoop(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Grid search algorithm for optimization.").add< GridSearchOptimizationLoop >());
}

void GridSearchOptimizationLoop::_initialize()
{
    m_gridSize = 1;
    for (const auto & parameter : l_parameters)
    {
        const auto resolution = parameter->getVectorFromData("resolution");
        const auto size = parameter->getVectorSize();
        for (unsigned long i = 0; i < size; i++)
            m_gridSize *= static_cast<int>(resolution[i]);
    }

    if (d_maxOptimizationSteps.getValue() == 0)
        d_maxOptimizationSteps.setValue(m_gridSize);

    d_startWithUpdate.setValue(true);
}

void GridSearchOptimizationLoop::_updateParameters()
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
        const auto lowerBound = parameter->getVectorFromData("lowerBound");
        const auto upperBound = parameter->getVectorFromData("upperBound");
        const auto resolution = parameter->getVectorFromData("resolution");
        const auto size = parameter->getVectorSize();

        type::vector<SReal> nextValue(size);
        for (unsigned long i = size; i > 0; i--)
        {
            const auto parameter_index = global_index % static_cast<int>(resolution[i-1]);
            nextValue[i-1] = lowerBound[i-1] + parameter_index * (upperBound[i-1] - lowerBound[i-1]) / (resolution[i-1] - 1);
            global_index = (global_index - parameter_index) / static_cast<int>(resolution[i-1]);
        }

        parameter->setDataFrom("nextValue", nextValue);
    }
}

}
