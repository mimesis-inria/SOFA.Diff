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

class SOFA_SOFADIFF_API BaseParameter: public core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(BaseParameter, core::objectmodel::BaseObject);

    virtual void resetGradient() = 0;
    virtual const type::vector<SReal> & getValueVector() = 0;
    virtual const type::vector<SReal> & getGradientVector() = 0;
    virtual void setValueVector(const type::vector<SReal> & vector) = 0;

    void parse(core::objectmodel::BaseObjectDescription *arg) override;
    bool hasHyperparameter(const std::string &hyperparameterName) const;
    SReal getHyperparameter(const std::string &hyperparameterName) const;

private:
    // To prevent memory leak
    std::vector<std::unique_ptr<Data<SReal>>> m_dynamicallyCreatedData;
};


template <class T>
class SOFA_SOFADIFF_API Parameter: public BaseParameter
{
public:
    SOFA_CLASS(Parameter, BaseParameter);

    Data<T> d_value;
    Data<T> d_gradient;

protected:
    type::vector<SReal> m_value;
    type::vector<SReal> m_gradient;

    virtual type::vector<SReal> getVectorFromData(const Data<T> & data);
    virtual void setDataFromVector(Data<T> & data, const type::vector<SReal> &vector);

public:
    Parameter();

    static std::string GetCustomClassName();

    void resetGradient() override;
    const type::vector<SReal> & getValueVector() override;
    const type::vector<SReal> & getGradientVector() override;
    void setValueVector(const type::vector<SReal>& vector) override;
};

}
