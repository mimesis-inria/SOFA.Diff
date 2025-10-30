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

#include <sofa/core/objectmodel/BaseObject.h>

namespace sofadiff
{

class SOFA_SOFADIFF_API TrainableParameter: public core::objectmodel::BaseObject
{
public:
    virtual void resetGradient() = 0;
    virtual void appendValueTo(std::vector<SReal> & vector) = 0;
    virtual void appendGradientTo(std::vector<SReal> & vector) = 0;

    virtual void setValueFrom(const std::vector<SReal> & vector, unsigned long index) = 0;
    virtual unsigned long getSize() = 0;
};


template <class T>
class SOFA_SOFADIFF_API TrainableParameterTemplated: public TrainableParameter
{
public:
    TrainableParameterTemplated();
    void init() override;

    Data<T> d_value;
    Data<T> d_gradient;
};


class SOFA_SOFADIFF_API TrainableParameterVector: public TrainableParameterTemplated<type::vector<SReal>>
{
public:
    void resetGradient() override
    {
        auto gradientAccessor = helper::getWriteAccessor(d_gradient);
        for (auto & v: gradientAccessor)
            v = 0;
    }
    void appendValueTo(std::vector<SReal> & vector) override
    {
        for (const auto & v : d_value.getValue())
            vector.push_back(v);
    }
    void appendGradientTo(std::vector<SReal> & vector) override
    {
        for (const auto & v : d_gradient.getValue())
            vector.push_back(v);
    }

    void setValueFrom(const std::vector<SReal> & vector, unsigned long index) override
    {
        auto valueAccessor = helper::getWriteAccessor(d_value);
        for (unsigned long i = 0; i < valueAccessor.size(); i++)
        {
            valueAccessor[i] = vector[index + i];
        }
    }

    unsigned long getSize() override
    {
        return d_value.getValue().size();
    }
};

}
