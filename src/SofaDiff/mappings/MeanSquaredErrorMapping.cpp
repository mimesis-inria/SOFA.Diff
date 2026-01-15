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
#include <SofaDiff/mappings/MeanSquaredErrorMapping.h>

#include <sofa/core/ObjectFactory.h>

namespace sofadiff
{

using namespace defaulttype;


void MeanSquaredErrorMapping::apply( const core::MechanicalParams* mparams, OutDataVecCoord& out, const InDataVecCoord& in)
{
    const auto values = helper::getReadAccessor(in);
    auto result = helper::getWriteAccessor(out);
    m_values.resize(values.size());
    for (unsigned long i = 0; i < m_values.size(); i++)
    {
        result[0][0] += values[i][0] * values[i][0];
        m_values[i] = values[i][0]; // Store the values for later use in applyJT
    }
}

void MeanSquaredErrorMapping::applyJ( const core::MechanicalParams* mparams, OutDataVecDeriv& out, const InDataVecDeriv& in)
{

}

void MeanSquaredErrorMapping::applyJT( const core::MechanicalParams* mparams, InDataVecDeriv& out, const OutDataVecDeriv& in)
{
    const auto state = this->getFrom();

    const auto values = helper::getReadAccessor(in);
    auto result = helper::getWriteAccessor(out);
    for (unsigned long i = 0; i < m_values.size(); i++)
    {
        result[i][0] += 2 * values[0][0] * m_values[i];
    }
    // TODO: am I supposed to multiply by kfactor?
}

void registerMeanSquaredErrorMapping(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Mapping values to the mean of their squares.")
        .add<MeanSquaredErrorMapping>());
}

}