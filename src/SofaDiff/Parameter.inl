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

#include <SofaDiff/Parameter.h>

namespace sofadiff
{

template<class T>
Parameter<T>::Parameter():
    d_value(initData(&d_value, "value", "value of the parameter")),
    d_gradient(initData(&d_gradient, "gradient", "Gradient of the parameter"))
{}

// ==============================================================================
// Operations on the default Data<T>
// ==============================================================================
template<class T>
void Parameter<T>::setValue(const SReal & value)
{
    this->setDataFromVector(d_value, type::vector<SReal>(this->getVectorSize(), value));
}

template<class T>
void Parameter<T>::setValue(const type::vector<SReal> & value)
{
    this->setDataFromVector(d_value, value);
}

template<class T>
const type::vector<SReal> & Parameter<T>::getValue() const
{
    return this->getVectorFromData(d_value);
}

template<class T>
void Parameter<T>::setGradient(const SReal & value)
{
    this->setDataFromVector(d_gradient, type::vector<SReal>(this->getVectorSize(), value));
}

template<class T>
void Parameter<T>::setGradient(const type::vector<SReal> & value)
{
    this->setDataFromVector(d_gradient, value);
}

template<class T>
const type::vector<SReal> & Parameter<T>::getGradient() const
{
    return this->getVectorFromData(d_gradient);
}


// ==============================================================================
// Operations on data in an arbitrary group
// ==============================================================================
template<class T>
void Parameter<T>::newDataInGroup(const std::string & dataName, const std::string & dataGroup)
{
    auto name = this->getDataInGroupFullName(dataName, dataGroup);
    auto * data = new Data<T>();
    data->setName(name);
    data->setGroup(dataGroup);
    this->addData(data, name);
    m_dataInGroups.emplace_back(data);
}

template<class T>
Data<T> * Parameter<T>::getDataPointer(const std::string &dataName, const std::string &dataGroup) const
{
    const auto name = this->getDataInGroupFullName(dataName, dataGroup);
    auto * baseData = this->findData(name);
    if (baseData == nullptr || baseData->getGroup() != dataGroup)
    {
        msg_error() << "Does not have Data '" << dataName << "' in group '" << dataGroup << "'.";
        return nullptr;
    }
    auto * data = dynamic_cast<Data<T>*>(baseData);
    if (data == nullptr)
    {
        msg_error() << "Data '" << dataName << "' in group '" << dataGroup << "' is not of the correct type";
        return nullptr;
    }
    return data;
}


template<class T>
void Parameter<T>::setDataInGroupFromConstant(const std::string & dataName, const std::string & dataGroup, SReal constant)
{
    this->setDataInGroup(dataName, dataGroup, type::vector(this->getVectorSize(), constant));
}

template<class T>
void Parameter<T>::setDataInGroupFromVector(const std::string & dataName, const std::string & dataGroup, const type::vector<SReal> & vector)
{
    auto * data = this->getDataPointer(dataName, dataGroup);
    if (data == nullptr)  // Note: getDataPointer() already printed an error message
        return;
    this->setDataFromVector(*data, vector);
}

template<class T>
const type::vector<SReal> & Parameter<T>::getVectorFromDataInGroup(const std::string & dataName, const std::string & dataGroup) const
{
    const auto * data = this->getDataPointer(dataName, dataGroup);
    if (data == nullptr)  // Note: getDataPointer() already printed an error message
        return type::vector<SReal>();
    return this->getVectorFromData(*data);
}

template<class T>
void Parameter<T>::setValueFromDataInGroup(const std::string & dataName, const std::string & dataGroup)
{
    auto * data = this->getDataPointer(dataName, dataGroup);
    if (data == nullptr)  // Note: getDataPointer() already printed an error message
        return;
    d_value.setValue(data->getValue());
}

template<class T>
void Parameter<T>::setDataInGroupFromValue(const std::string & dataName, const std::string & dataGroup)
{
    auto * data = this->getDataPointer(dataName, dataGroup);
    if (data == nullptr)  // Note: getDataPointer() already printed an error message
        return;
    data->setValue(d_value.getValue());
}

}
