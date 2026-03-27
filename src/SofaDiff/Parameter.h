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
    void parse(core::objectmodel::BaseObjectDescription *arg) override;

    virtual size_t getVectorSize() const = 0;

    // Operations on the default Data — implemented in template class because needs access to said Data
    virtual void setValue(const SReal & constant) = 0;
    virtual void setValue(const type::vector<SReal> & vector) = 0;
    virtual const type::vector<SReal> & getValue() const = 0;

    virtual void setGradient(const SReal & constant) = 0;
    virtual void setGradient(const type::vector<SReal> & vector) = 0;
    virtual const type::vector<SReal> & getGradient() const = 0;

    // Operations on Data in an arbitrary group
    std::string getDataInGroupFullName(const std::string & dataName, const std::string & dataGroup) const;

    core::BaseData * findDataInGroup(const std::string & dataName, const std::string & dataGroup) const;
    virtual void newDataInGroup(const std::string & dataName, const std::string & dataGroup) = 0;

    virtual void setDataInGroupFromConstant(const std::string & dataName, const std::string & dataGroup, SReal constant) = 0;
    virtual void setDataInGroupFromVector(const std::string & dataName, const std::string & dataGroup, const type::vector<SReal> & vector) = 0;
    virtual const type::vector<SReal> & getVectorFromDataInGroup(const std::string & dataName, const std::string & dataGroup) const = 0;

    virtual void setValueFromDataInGroup(const std::string & dataName, const std::string & dataGroup) = 0;
    virtual void setDataInGroupFromValue(const std::string & dataName, const std::string & dataGroup) = 0;

    // Operations on Data in the hyperparameters group
    bool hasHyperparameter(const std::string & dataName) const;
    void newHyperparameter(const std::string & dataName);
    void setHyperparameter(const std::string & dataName, SReal constant);
    void setHyperparameter(const std::string & dataName, const std::vector<SReal> & vector);
    const std::vector<SReal> & getHyperparameter(const std::string & dataName) const;
protected:
    static constexpr std::string m_hyperparametersGroup = "Hyperparameters";
};


template <class T>
class SOFA_SOFADIFF_API Parameter: public BaseParameter
{
public:
    SOFA_CLASS(Parameter, BaseParameter);

    Parameter();
    Data<T> d_value;
    Data<T> d_gradient;

    // Operations on the default Data
    void setValue(const SReal & value) override;
    void setValue(const type::vector<SReal> & value) override;
    const type::vector<SReal> & getValue() const override;

    void setGradient(const SReal & value) override;
    void setGradient(const type::vector<SReal> & value) override;
    const type::vector<SReal> & getGradient() const override;

    // Operations on data in an arbitrary group
    Data<T> * getDataPointer(const std::string & dataName, const std::string & dataGroup) const;

    void newDataInGroup(const std::string & dataName, const std::string & dataGroup) override;

    void setDataInGroupFromConstant(const std::string & dataName, const std::string & dataGroup, SReal constant) override;
    void setDataInGroupFromVector(const std::string & dataName, const std::string & dataGroup, const type::vector<SReal> & vector) override;
    const type::vector<SReal> & getVectorFromDataInGroup(const std::string & dataName, const std::string & dataGroup) const override;

    void setValueFromDataInGroup(const std::string & dataName, const std::string & dataGroup) override;
    void setDataInGroupFromValue(const std::string & dataName, const std::string & dataGroup) override;

private:
    type::vector<std::unique_ptr<Data<T>>> m_dataInGroups;

    // Methods to be specialized
    static std::string GetCustomClassName();
    void setDataFromVector(Data<T> & data, const type::vector<SReal> &vector);
    const type::vector<SReal> & getVectorFromData(const Data<T> & data) const;
    size_t getVectorSize() const override;
};

}
