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

#include <sofa/diff/config.h>

#include <sofa/core/Mapping.h>
// #include <sofa/linearalgebra/EigenSparseMatrix.h>
// #include <sofa/type/Mat.h>
// #include <sofa/type/Vec.h>
// #include <sofa/type/RGBAColor.h>
// #include <sofa/defaulttype/RigidTypes.h>

namespace sofadiff
{

class MeanSquaredErrorMapping : public sofa::core::Mapping<sofa::defaulttype::Vec1Types, sofa::defaulttype::Vec1Types>
{
public:
    SOFA_CLASS(MeanSquaredErrorMapping, SOFA_TEMPLATE2(sofa::core::Mapping, defaulttype::Vec1Types, defaulttype::Vec1Types));

    virtual void apply( const core::MechanicalParams* mparams, OutDataVecCoord& out, const InDataVecCoord& in) override;
    virtual void applyJ( const core::MechanicalParams* mparams, OutDataVecDeriv& out, const InDataVecDeriv& in) override;
    virtual void applyJT( const core::MechanicalParams* mparams, InDataVecDeriv& out, const OutDataVecDeriv& in) override;

    type::vector<SReal> m_values;
};

}