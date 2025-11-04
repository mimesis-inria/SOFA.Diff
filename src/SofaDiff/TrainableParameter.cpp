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
#include <SofaDiff/TrainableParameter.inl>

#include <sofa/core/ObjectFactory.h>


namespace sofadiff
{

void TrainableParameter::parse(core::objectmodel::BaseObjectDescription *arg)
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


void registerTrainableParameter(core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Trainable parameter of vector type.").add< TrainableParameterVector >());
    factory->registerObjects(core::ObjectRegistrationData("Trainable parameter of scalar type.").add< TrainableParameterScalar >());
}


}
