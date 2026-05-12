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
#include <sofa/diff/optimizers/GradientDescentOptimizationLoop.h>

#include <sofa/core/ObjectFactory.h>


using namespace sofa::core;

namespace sofadiff
{
using namespace sofa::core::objectmodel;

void registerGradientDescentOptimizationLoop(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Gradient descent algorithm for optimization.").add< GradientDescentOptimizationLoop >());
}

void GradientDescentOptimizationLoop::_updateParameters()
{
    for (auto & parameter : l_parameters)
    {
        auto nextValue = parameter->getVectorFromData("value");

        const auto learningRate = parameter->getVectorFromData("learningRate");
        const auto gradient = parameter->getVectorFromData("gradient");
        for (size_t j = 0; j < gradient.size(); j++)
            nextValue[j] -= learningRate[j] * gradient[j];

        if (parameter->hasData("lowerBound"))
        {
            const auto lowerBound = parameter->getVectorFromData("lowerBound");
            for (size_t j = 0; j < gradient.size(); j++)
                if (nextValue[j] < lowerBound[j])
                    nextValue[j] = lowerBound[j];
        }

        if (parameter->hasData("upperBound"))
        {
            const auto upperBound = parameter->getVectorFromData("upperBound");
            for (size_t j = 0; j < gradient.size(); j++)
                if (nextValue[j] > upperBound[j])
                    nextValue[j] = upperBound[j];
        }

        parameter->setDataFrom("nextValue", nextValue);
    }
}

}