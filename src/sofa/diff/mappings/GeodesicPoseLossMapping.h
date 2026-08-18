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

#include <sofa/diff/config.h>

#include <sofa/core/Mapping.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/type/vector.h>
#include <sofa/type/Vec.h>

namespace sofadiff
{

template<class DataTypes>
class GeodesicPoseLossMapping :
    public sofa::core::Mapping<DataTypes, sofa::defaulttype::Vec1Types>
{
public:
    using Inherit = sofa::core::Mapping<DataTypes, sofa::defaulttype::Vec1Types>;

    using Real = typename DataTypes::Real;
    using Coord = typename DataTypes::Coord;
    using Deriv = typename DataTypes::Deriv;
    using VecCoord = typename DataTypes::VecCoord;
    using VecDeriv = typename DataTypes::VecDeriv;

    using InDataVecCoord = sofa::Data<VecCoord>;
    using InDataVecDeriv = sofa::Data<VecDeriv>;

    using OutTypes = sofa::defaulttype::Vec1Types;
    using OutDataVecCoord = sofa::Data<typename OutTypes::VecCoord>;
    using OutDataVecDeriv = sofa::Data<typename OutTypes::VecDeriv>;

    static constexpr unsigned int N = Deriv::size();

    SOFA_CLASS(
        SOFA_TEMPLATE(GeodesicPoseLossMapping, DataTypes),
        SOFA_TEMPLATE2(sofa::core::Mapping, DataTypes, sofa::defaulttype::Vec1Types)
    );

public:
    GeodesicPoseLossMapping();

    void apply(
        const sofa::core::MechanicalParams* mparams,
        OutDataVecCoord& out,
        const InDataVecCoord& in) override;

    void applyJ(
        const sofa::core::MechanicalParams* mparams,
        OutDataVecDeriv& out,
        const InDataVecDeriv& in) override;

    void applyJT(
        const sofa::core::MechanicalParams* mparams,
        InDataVecDeriv& out,
        const OutDataVecDeriv& in) override;

protected:
    sofa::type::Vec<3, Real> positionOf(const Coord& coord) const;

    sofa::type::Vec<3, Real> targetPosition(
        const sofa::type::vector<SReal>& target,
        sofa::Index base) const;

    sofa::type::Vec<3, Real> rotationError(
        const Coord& coord,
        const sofa::type::vector<SReal>& target,
        sofa::Index base) const;

    sofa::Index targetStride() const;

    sofa::Index targetBaseForIndex(
        sofa::Index index,
        sofa::Size inputSize,
        sofa::Size targetSize) const;

protected:
    sofa::Data<sofa::type::vector<SReal>> d_targetPose;
    sofa::Data<SReal> d_positionWeight;
    sofa::Data<SReal> d_rotationWeight;

    sofa::type::vector<Deriv> m_inputGradient;
};

void registerGeodesicPoseLossMapping(sofa::core::ObjectFactory* factory);

} // namespace sofadiff