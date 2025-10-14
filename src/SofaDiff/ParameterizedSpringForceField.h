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

#include <SofaDiff/CostFunctionMapping.h>
#include <SofaDiff/ParameterizedForceField.h>

#include <sofa/core/behavior/StateAccessor.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/component/solidmechanics/spring/SpringForceField.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/behavior/MultiVec.h>


namespace sofadiff
{

using namespace sofa::component::solidmechanics::spring;
using namespace sofa::defaulttype;

class ParameterizedSpringForceField :
    public ParameterizedForceField,
    public SpringForceField<Vec3Types>
{
public:
    SOFA_CLASS2(ParameterizedSpringForceField, ParameterizedForceField, SpringForceField<Vec3Types>);

    void applyParametersJacobianTranspose(const core::MechanicalParams* mparams, core::MultiVecDerivId vecId) override;

private:
    Real m_derivative;
};

}
