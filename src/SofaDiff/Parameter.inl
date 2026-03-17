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
#include <SofaDiff/Parameter.h>

namespace sofadiff
{

template<class T>
Parameter<T>::Parameter()
: d_value(initData(&d_value, "value", "Value of the parameter"))
, d_nextValue(initData(&d_nextValue, "nextValue", "Next value of the parameter (during optimization)"))
, d_gradient(initData(&d_gradient, "gradient", "Gradient of the parameter"))
{}

template<class T>
void Parameter<T>::resetGradient()
{
    // Not optimal, but works for all T
    const auto vector = getVectorFromData(d_value);
    const type::vector<SReal> gradient(vector.size(), 0.0);
    setDataFromVector(d_gradient, gradient);
}

template<class T>
const type::vector<SReal> & Parameter<T>::getValueVector()
{
    return getVectorFromData(d_value);
}

template<class T>
const type::vector<SReal> & Parameter<T>::getNextValueVector()
{
    return getVectorFromData(d_nextValue);;
}

template<class T>
const type::vector<SReal> & Parameter<T>::getGradientVector()
{
    return getVectorFromData(d_gradient);
}

template<class T>
void Parameter<T>::setValueVector(const type::vector<SReal>& vector)
{
    setDataFromVector(d_value, vector);
}

template<class T>
void Parameter<T>::setNextValueVector(const type::vector<SReal>& vector)
{
    setDataFromVector(d_nextValue, vector);
}

}
