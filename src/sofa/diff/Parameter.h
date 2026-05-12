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
#pragma once

#include <sofa/diff/config.h>

#include <sofa/core/objectmodel/BaseObject.h>

namespace sofadiff
{

class SOFA_SOFADIFF_API BaseParameter: public core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(BaseParameter, core::objectmodel::BaseObject);
    void parse(core::objectmodel::BaseObjectDescription * arg) override;
    void addHyperparameter(const std::string & name, const std::string & value);

    void enterGroup(const std::string & groupName);
    void leaveGroup();

    bool hasData(const std::string & dataName) const;

    virtual core::BaseData * newData(const std::string & dataName) = 0;
    virtual void setDataFrom(const std::string & dataName, const std::string & fromDataName) = 0;
    virtual void setDataFrom(const std::string & dataName, const type::vector<SReal> & fromVector) = 0;
    virtual void setDataFrom(const std::string & dataName, SReal fromConstant) = 0;
    virtual type::vector<SReal> getVectorFromData(const std::string & dataName) const = 0;
    virtual size_t getVectorSize() const = 0;

protected:
    type::vector<std::string> m_groupStack;
    std::string getFullName(std::string dataName) const;
};


template <class T>
class SOFA_SOFADIFF_API Parameter: public BaseParameter
{
public:
    SOFA_CLASS(Parameter, BaseParameter);

// Default Data
    Parameter();
    Data<T> d_value;
    Data<T> d_gradient;

// Generic implementation of the abstract interface
    core::BaseData * newData(const std::string & dataName) override;
    void setDataFrom(const std::string & dataName, const std::string & fromDataName) override;
    void setDataFrom(const std::string & dataName, const type::vector<SReal> & vector) override;
    void setDataFrom(const std::string & dataName, SReal constant) override;
    type::vector<SReal> getVectorFromData(const std::string & dataName) const override;
private:
    Data<T> * getData(const std::string & dataName) const;
    type::vector<std::unique_ptr<Data<T>>> m_dataInGroups;

// Methods that require specialized implementation
public:
    size_t getVectorSize() const override;
    static std::string GetCustomClassName();
private:
    void vectorToData(Data<T> & data, const type::vector<SReal> & vector);
    type::vector<SReal> dataToVector(const Data<T> & data) const;
};

}
