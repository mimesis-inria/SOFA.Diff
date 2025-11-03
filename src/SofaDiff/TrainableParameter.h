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
    TrainableParameter();

    virtual void resetGradient() = 0;
    virtual const type::vector<SReal> & getValueVector() = 0;
    virtual const type::vector<SReal> & getGradientVector() = 0;
    virtual void setValueVector(const type::vector<SReal> & vector) = 0;

    Data<SReal> d_learningRate;
};


template <class T>
class SOFA_SOFADIFF_API TrainableParameterTemplated: public TrainableParameter
{
public:
    TrainableParameterTemplated();

    Data<T> d_value;
    Data<T> d_gradient;
};


class SOFA_SOFADIFF_API TrainableParameterVector: public TrainableParameterTemplated<type::vector<SReal>>
{
public:

    void init() override
    {
        // TODO: is it the right way of doing this? Should I be doing this? (goal: allowing accumulation from ParameterizedForceField)
        auto gradientAccessor = helper::getWriteOnlyAccessor(d_gradient);
        gradientAccessor.resize(d_value.getValue().size());
    }

    void resetGradient() override
    {
        auto gradientAccessor = helper::getWriteAccessor(d_gradient);
        for (auto & v: gradientAccessor)
            v = 0;
    }

    const type::vector<SReal> & getValueVector() override
    {
        return d_value.getValue();
    }

    const type::vector<SReal> & getGradientVector() override
    {
        return d_gradient.getValue();
    }

    void setValueVector(const type::vector<SReal> & vector) override
    {
        d_value.setValue(vector);
    }
};


class SOFA_SOFADIFF_API TrainableParameterScalar: public TrainableParameterTemplated<SReal>
{
private:
    type::vector<SReal> m_value;
    type::vector<SReal> m_gradient;

public:
    void init() override
    {
        // TODO: le init est bien appelé qu’une fois? Comment initialiser un type::vector sinon ?
        m_value.push_back(0);
        m_gradient.push_back(0);
    }

    void resetGradient() override
    {
        d_gradient.setValue(0);
    }

    const type::vector<SReal> & getValueVector() override
    {
        m_value[0] = d_value.getValue();
        return m_value;
    }

    const type::vector<SReal> & getGradientVector() override
    {
        m_gradient[0] = d_gradient.getValue();
        return m_gradient;
    }

    void setValueVector(const type::vector<SReal> & vector) override
    {
        d_value.setValue(vector[0]);
    }
};

}
