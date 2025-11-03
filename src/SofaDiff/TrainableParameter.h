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
    Data<T> d_value;
    Data<T> d_gradient;

protected:
    type::vector<SReal> m_value;
    type::vector<SReal> m_gradient;

    virtual type::vector<SReal> getVectorFromData(const Data<T> & data) = 0;
    virtual void setDataFromVector(Data<T> & data, const type::vector<SReal> &vector) = 0;

public:
    TrainableParameterTemplated();

    void resetGradient() override
    {
        // Not optimal, but works for all T
        const auto vector = getVectorFromData(d_value);
        const type::vector<SReal> gradient(vector.size(), 0.0);
        setDataFromVector(d_gradient, gradient);
    }

    const type::vector<SReal> & getValueVector() override
    {
        m_value = getVectorFromData(d_value);
        return m_value;
    }

    const type::vector<SReal> & getGradientVector() override
    {
        m_gradient = getVectorFromData(d_gradient);
        return m_gradient;
    }

    void setValueVector(const type::vector<SReal>& vector) override
    {
        setDataFromVector(d_value, vector);
    }
};


class SOFA_SOFADIFF_API TrainableParameterVector: public TrainableParameterTemplated<type::vector<SReal>>
{
public:
    type::vector<SReal> getVectorFromData(const Data<type::vector<SReal>> & data) override
    {
        return data.getValue();
    }

    void setDataFromVector(Data<type::vector<SReal>> & data, const type::vector<SReal> & vector) override
    {
        data.setValue(vector);
    }
};


class SOFA_SOFADIFF_API TrainableParameterScalar: public TrainableParameterTemplated<SReal>
{
public:
    type::vector<SReal> getVectorFromData(const Data<SReal> & data) override
    {
        return {data.getValue()};
    }

    void setDataFromVector(Data<SReal> & data, const type::vector<SReal> & vector) override
    {
        data.setValue(vector[0]);
    }
};

}
