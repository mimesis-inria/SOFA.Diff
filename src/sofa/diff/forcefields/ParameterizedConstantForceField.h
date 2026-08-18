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

#include <sofa/diff/ParameterizedForceField.h>
#include <sofa/component/mechanicalload/ConstantForceField.h>
#include <sofa/core/behavior/StateAccessor.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/behavior/MultiVec.h>


namespace sofadiff
{

using namespace sofa::defaulttype;
using namespace sofa::component::mechanicalload;

class ParameterizedConstantForceField:
    public Parameterized,
    public ConstantForceField<Rigid3Types>
{
public:
    SOFA_CLASS(ParameterizedConstantForceField, ConstantForceField<Rigid3Types>);

    typedef typename DataTypes::VecDeriv VecDeriv;

    ParameterizedConstantForceField();

    // TODO: Is a pointer the correct type to use here?
    Parameter< VecDeriv >* m_forceParameter;
    Data< sofa::type::fixed_array<int,6> > d_componentFlags;

    void init() override;

    void applyParametersJacobianTranspose(const core::MechanicalParams* mparams, core::MultiVecDerivId vecId) override;

    void addForce(const core::MechanicalParams* params,
              DataVecDeriv& f,
              const DataVecCoord& x,
              const DataVecDeriv& v) override;
};

}