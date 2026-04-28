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
#pragma once

#include <SofaDiff/config.h>

#include <sofa/core/MultiVecId.h>
#include <sofa/core/State.h>
#include <sofa/core/trait/DataTypes.h>


namespace sofadiff
{

class LossState: public core::State<defaulttype::Vec1Types>
{
public:
    SOFA_CLASS(LossState, core::State<defaulttype::Vec1Types>);

    Data< VecCoord_t<defaulttype::Vec1Types> > d_value;
    Data< VecDeriv_t<defaulttype::Vec1Types> > d_gradient;

    LossState();
    void setGradientVecId(core::MultiVecDerivId & gradientVecId) const;

    Data< VecCoord >* write(core::VecCoordId v) override;
    const Data< VecCoord >* read(core::ConstVecCoordId v) const override;

    Data< VecDeriv >* write(core::VecDerivId v) override;
    const Data< VecDeriv >* read(core::ConstVecDerivId v) const override;

    Data< MatrixDeriv >* write(core::MatrixDerivId v) override;
    const Data< MatrixDeriv >* read(core::ConstMatrixDerivId v) const override;

    Size getSize() const override;
    void resize(Size vsize) override;

};

}
