/******************************************************************************
 *                 SOFA, Simulation Open-Framework Architecture                *
 *                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
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
 * Authors: The SOFA Team and external contributors (see Authors.txt)          *
 *                                                                             *
 * Contact information: contact@sofa-framework.org                             *
 ******************************************************************************/
#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/TrainableParameter.h>

namespace sofadiff
{

template<class T>
TrainableParameterTemplated<T>::TrainableParameterTemplated()
: d_value(initData(&d_value, "value", "Value of the parameter"))
, d_gradient(initData(&d_gradient, "gradient", "Gradient of the parameter"))
{}

template<class T>
void TrainableParameterTemplated<T>::init()
{
    // TODO: is it the right way of doing this? Should I be doing this? (goal: allowing accumulation from ParameterizedForceField)
    auto gradientAccessor = helper::getWriteOnlyAccessor(d_gradient);
    gradientAccessor.resize(d_value.getValue().size());
    // TODO: the gradient needs to be reset at some point!
}


}
