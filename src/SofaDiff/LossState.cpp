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

#include <SofaDiff/LossState.h>
#include <SofaDiff/GradientDescentController.h>

#include <sofa/core/State.h>
#include <sofa/core/trait/DataTypes.h>
#include <sofa/core/ObjectFactory.h>


namespace sofadiff
{

LossState::LossState()
: d_value(initData(&d_value, "value", "Value of the loss"))
, d_gradient(initData(&d_gradient, "gradient", "Gradient of the loss"))
{
    s_geometricGradientId.setId(this, core::TVecId<core::VecType::V_DERIV, core::VecAccess::V_WRITE>(6));

    d_value.beginWriteOnly()->resize( 1 );
    d_value.endEdit();

    d_gradient.beginWriteOnly()->resize( 1 );
    d_gradient.endEdit();
}

Data< VecCoord_t<defaulttype::Vec1Types> >* LossState::write(core::VecCoordId v)
{
    if (v.index == 1)
        return &d_value;
    return nullptr;
}

const Data< VecCoord_t<defaulttype::Vec1Types> >* LossState::read(core::ConstVecCoordId v) const
{
    if (v.index == 1)
        return &d_value;
    return nullptr;
}

Data< VecDeriv_t<defaulttype::Vec1Types> >* LossState::write(core::VecDerivId v)
{
    if (v.index == 6)
        return &d_gradient;
    return nullptr;
}

const Data< VecDeriv_t<defaulttype::Vec1Types> >* LossState::read(core::ConstVecDerivId v) const
{
    if (v.index == 6)
        return &d_gradient;
    return nullptr;
}

Data< MatrixDeriv_t<defaulttype::Vec1Types> >* LossState::write(core::MatrixDerivId v)
{
    SOFA_UNUSED(v);
    return nullptr;
}

const Data< MatrixDeriv_t<defaulttype::Vec1Types> >* LossState::read(core::ConstMatrixDerivId v) const
{
    SOFA_UNUSED(v);
    return nullptr;
}

Size LossState::getSize() const
{
 return 1;
}

void LossState::resize(Size vsize)
{
    SOFA_UNUSED(vsize);
}


void registerLossState(core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("State container for loss objects.").add< LossState >());
}

}