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
    SOFA_CLASS(TrainableParameter, core::objectmodel::BaseObject);

    virtual void resetGradient() = 0;
    virtual const type::vector<SReal> & getValueVector() = 0;
    virtual const type::vector<SReal> & getGradientVector() = 0;
    virtual void setValueVector(const type::vector<SReal> & vector) = 0;

    void parse(core::objectmodel::BaseObjectDescription *arg) override;

private:
    // To prevent memory leak
    std::vector<std::unique_ptr<Data<SReal>>> m_dynamicallyCreatedData;
};


template <class T>
class SOFA_SOFADIFF_API TrainableParameterTemplated: public TrainableParameter
{
public:
    SOFA_CLASS(TrainableParameterTemplated, TrainableParameter);

    Data<T> d_value;
    Data<T> d_gradient;

protected:
    type::vector<SReal> m_value;
    type::vector<SReal> m_gradient;

    virtual type::vector<SReal> getVectorFromData(const Data<T> & data);
    virtual void setDataFromVector(Data<T> & data, const type::vector<SReal> &vector);

public:
    TrainableParameterTemplated();

    void resetGradient() override;
    const type::vector<SReal> & getValueVector() override;
    const type::vector<SReal> & getGradientVector() override;
    void setValueVector(const type::vector<SReal>& vector) override;
};


class SOFA_SOFADIFF_API TrainableParameterVector: public TrainableParameterTemplated<type::vector<SReal>>
{
public:
    SOFA_CLASS(TrainableParameterVector, TrainableParameterTemplated<type::vector<SReal>>);
};


class SOFA_SOFADIFF_API TrainableParameterScalar: public TrainableParameterTemplated<SReal>
{
public:
    SOFA_CLASS(TrainableParameterScalar, TrainableParameterTemplated<SReal>);
};

}
