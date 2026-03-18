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

#include <SofaDiff/config.h>
#include <SofaDiff/Parameter.inl>

#include <sofa/core/ObjectFactory.h>


namespace sofadiff
{

void BaseParameter::parse(core::objectmodel::BaseObjectDescription *arg)
{
    BaseObject::parse(arg);
    const auto map = arg->getAttributeMap();
    for (const auto&[name, attribute] : map)
    {
        if (!attribute.isAccessed())
        {
            auto * data = new Data<SReal>(std::stod(attribute.c_str())); // Silently ignores additional values
            data->setName(name);
            this->addData(data, name);
            m_dynamicallyCreatedData.emplace_back(data);
            arg->removeAttribute(name);
        }
    }
}

bool BaseParameter::hasHyperparameter(const std::string &hyperparameterName) const
{
    const auto * baseData = this->findData(hyperparameterName);
    return baseData != nullptr;
}

SReal BaseParameter::getHyperparameter(const std::string &hyperparameterName) const
{
    const auto * baseData = this->findData(hyperparameterName);
    if (baseData == nullptr)
    {
        msg_error() << "Parameter " << this->getName() << " does not have hyperparameter " << hyperparameterName;
        return 0.0;
    }

    const auto * data = dynamic_cast<const Data<SReal>*>(baseData);
    if (data == nullptr)
    {
        // Unlikely to trigger since the parsing converts the string value of the hyperparameter to a double with std::stod()
        msg_error() << "Hyperparameter " << hyperparameterName << " of parameter " << this->getName() << " is not a scalar";
        return 0.0;
    }

    return data->getValue();
}


template<>
std::string Parameter<type::vector<SReal>>::GetCustomClassName()
{
    return "TrainableParameterVector";
}

template<>
const type::vector<SReal>& Parameter<type::vector<SReal>>::getVectorFromData(const Data<type::vector<SReal>> & data) const
{
    return data.getValue();
}

template<>
void Parameter<type::vector<SReal>>::setDataFromVector(Data<type::vector<SReal>> & data, const type::vector<SReal> & vector)
{
    data.setValue(vector);
}


template<>
std::string Parameter<SReal>::GetCustomClassName()
{
    return "TrainableParameterScalar";
}

template<>
const type::vector<SReal>& Parameter<SReal>::getVectorFromData(const Data<SReal> & data) const
{
    return {data.getValue()};
}

template<>
void Parameter<SReal>::setDataFromVector(Data<SReal> & data, const type::vector<SReal> & vector)
{
    data.setValue(vector[0]);
}


void registerTrainableParameter(core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Trainable parameter of vector type.").add< Parameter<type::vector<SReal>> >());
    factory->registerObjects(core::ObjectRegistrationData("Trainable parameter of scalar type.").add< Parameter<SReal> >());
}


}
