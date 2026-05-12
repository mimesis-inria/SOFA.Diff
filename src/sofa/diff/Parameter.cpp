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
#include <sofa/diff/config.h>
#include <sofa/diff/Parameter.inl>

#include <sofa/core/ObjectFactory.h>


namespace sofadiff
{

// ==============================================================================
// Base class methods
// ==============================================================================
void BaseParameter::parse(core::objectmodel::BaseObjectDescription *arg)
{
    BaseObject::parse(arg);
    const auto map = arg->getAttributeMap();
    for (const auto&[name, attribute] : map)
    {
        if (attribute.isAccessed())
            continue;

        this->addHyperparameter(name, attribute);
        arg->removeAttribute(name);
    }
}

std::vector<double> parse_doubles(const std::string & s)
{
    std::istringstream iss(s);
    std::vector<double> values;
    double value;
    while (iss >> value)
        values.push_back(value);
    return values;
}

void BaseParameter::addHyperparameter(const std::string & name, const std::string & value)
{
    auto * data = this->newData(name);
    if (data == nullptr)
        return;

    const auto values = parse_doubles(value);
    if (values.size() == 1)
        this->setDataFrom(name, values[0]);
    else
        this->setDataFrom(name, values);

    data->setGroup("Hyperparameters");
}


void BaseParameter::enterGroup(const std::string & groupName)
{
    m_groupStack.push_back(groupName);
}

void BaseParameter::leaveGroup()
{
    m_groupStack.pop_back();
}

bool BaseParameter::hasData(const std::string & dataName) const
{
    return this->findData(dataName) != nullptr || this->findData(this->getFullName(dataName)) != nullptr;
}

std::string BaseParameter::getFullName(std::string dataName) const
{
    if (m_groupStack.empty())
        return dataName;
    return dataName + " [" + m_groupStack.back() + "]";
}


// ==============================================================================
// Specialization for type::vector<SReal>
// ==============================================================================
template<>
std::string Parameter<type::vector<SReal>>::GetCustomClassName()
{
    return "TrainableParameterVector";
}

template<>
void Parameter<type::vector<SReal>>::vectorToData(Data<type::vector<SReal>> & data, const type::vector<SReal> & vector)
{
    data.setValue(vector);
}

template<>
type::vector<SReal> Parameter<type::vector<SReal>>::dataToVector(const Data<type::vector<SReal>> & data) const
{
    return data.getValue();
}

template<>
size_t Parameter<type::vector<SReal>>::getVectorSize() const
{
    return d_value.getValue().size();
}


// ==============================================================================
// Specialization for SReal
// ==============================================================================
template<>
std::string Parameter<SReal>::GetCustomClassName()
{
    return "TrainableParameterScalar";
}

template<>
void Parameter<SReal>::vectorToData(Data<SReal> & data, const type::vector<SReal> & vector)
{
    data.setValue(vector[0]);
}

template<>
type::vector<SReal> Parameter<SReal>::dataToVector(const Data<SReal> & data) const
{
    return {data.getValue()};
}

template<>
size_t Parameter<double>::getVectorSize() const
{
    return 1;
}


// ==============================================================================
// Registering
// ==============================================================================
void registerTrainableParameter(core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Trainable parameter of vector type.").add< Parameter<type::vector<SReal>> >());
    factory->registerObjects(core::ObjectRegistrationData("Trainable parameter of scalar type.").add< Parameter<SReal> >());
}


}
