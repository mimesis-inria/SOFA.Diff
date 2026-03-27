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

template <class T>
core::BaseData * Parameter<T>::newData(const std::string & dataName)
{
    if (this->hasData(dataName))
    {
        msg_error() << "Data " << dataName << " already exists.";
        return nullptr;
    }

    auto * data = new Data<T>();
    auto fullName = this->getFullName(dataName);
    data->setName(fullName);  // TODO: try giving it the 'dataName' to see if there is collision (my guess: in the GUI yes)
    if (!m_groupStack.empty())
        data->setGroup(m_groupStack.back());
    this->addData(data, fullName);
    m_dataInGroups.emplace_back(data);
    return data;
}

template<class T>
Data<T> * Parameter<T>::getData(const std::string & dataName) const
{
    auto baseData = this->findData(dataName);
    if (baseData == nullptr)
        baseData = this->findData(this->getFullName(dataName));
    if (baseData == nullptr)
    {
        msg_error() << "No data named \"" << dataName << "\"";
        return nullptr;
    }
    auto data = dynamic_cast<Data<T>*>(baseData);
    if (data == nullptr)
    {
        msg_error() << "Data named \"" << dataName << "\" is not of the correct type";
        return nullptr;
    }
    return data;
}

template<class T>
void Parameter<T>::setDataFrom(const std::string &dataName, const std::string &fromDataName)
{
    auto data = this->getData(dataName);
    if (data == nullptr)
        return;
    auto fromData = this->getData(fromDataName);
    if (fromData == nullptr)
        return;
    data->setValue(fromData->getValue());
}

template <class T>
void Parameter<T>::setDataFrom(const std::string & dataName, const type::vector<SReal> &vector)
{
    auto data = this->getData(dataName);
    if (data == nullptr)
        return;
    this->vectorToData(*data, vector);
}

template <class T>
void Parameter<T>::setDataFrom(const std::string & dataName, const SReal constant)
{
    this->setDataFrom(dataName, type::vector<SReal>(this->getVectorSize(), constant));
}

template <class T>
type::vector<SReal> Parameter<T>::getVectorFromData(const std::string & dataName) const
{
    auto data = this->getData(dataName);
    if (data == nullptr)
        return {};
    return this->dataToVector(*data);
}

}
