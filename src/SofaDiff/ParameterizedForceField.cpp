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

#include <SofaDiff/ParameterizedForceField.h>
#include <SofaDiff/TrainableParameter.h>


namespace sofadiff
{

TrainableParameter * ParameterizedForceField::getParentParameter(const core::BaseData *data)
{
    const auto * parent = data->getParent();
    if (parent == nullptr)
        return nullptr;

    auto * parameter = dynamic_cast<TrainableParameter*>(parent->getOwner());
    if (parameter == nullptr)
        return nullptr;

    return parameter;
}

void ParameterizedForceField::initParameter(Data<type::vector<SReal>> & data, Data<type::vector<SReal> > * & gradientPtr)
{
    m_canBeTrained.insert(&data);

    auto * parameter = getParentParameter(&data);
    if (parameter == nullptr)
        return;
    auto * parameterVector = dynamic_cast<TrainableParameterVector*>(parameter);
    if (parameterVector == nullptr)
    {
        // TODO: cannot use msg_error()
        std::cout << "Error: " << data.getName() << " parameter must be a TrainableParameterVector" << std::endl;
    }
    gradientPtr = &parameterVector->d_gradient; // TODO: Do I need to check that d_gradient is not nullptr?
}

void ParameterizedForceField::initParameter(Data<SReal> & data, Data<SReal> * & gradientPtr)
{
    m_canBeTrained.insert(&data);
    if (auto * parameter = getParentParameter(&data); parameter != nullptr)
    {
        auto * parameterScalar = dynamic_cast<TrainableParameterScalar*>(parameter);
        if (parameterScalar == nullptr)
        {
            // TODO: cannot use msg_error()
            std::cout << "Error: " << data.getName() << " parameter must be a TrainableParameterScalar" << std::endl;
        }
        gradientPtr = &parameterScalar->d_gradient; // TODO: Do I need to check that d_gradient is not nullptr?
    }
}

void ParameterizedForceField::checkForNotImplementedParameters(const type::vector<core::BaseData *> &dataFields) const
{
    for (const auto & data : dataFields)
    {
        if (getParentParameter(data) != nullptr && !m_canBeTrained.contains(data))
        {
            // TODO: cannot use msg_error()
            std::cout << "Error: " << "Data " << data->getName() << " is not trainable.";
        }
    }
}


}
