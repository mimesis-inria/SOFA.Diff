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

#include <sofa/core/MultiMapping.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/type/vector.h>

namespace sofadiff
{

class MeanSquaredErrorMultiMapping: public sofa::core::MultiMapping<sofa::defaulttype::Vec1Types, sofa::defaulttype::Vec1Types>
{
public:
    SOFA_CLASS(MeanSquaredErrorMultiMapping,
               SOFA_TEMPLATE2(sofa::core::MultiMapping,
                             sofa::defaulttype::Vec1Types,
                             sofa::defaulttype::Vec1Types));

    virtual void apply(const sofa::core::MechanicalParams* mparams,
               const sofa::type::vector<OutDataVecCoord*>& dataVecOutPos,
               const sofa::type::vector<const InDataVecCoord*>& dataVecInPos) override;

    virtual void applyJ(const sofa::core::MechanicalParams* mparams,
                const sofa::type::vector<OutDataVecDeriv*>& dataVecOutVel,
                const sofa::type::vector<const InDataVecDeriv*>& dataVecInVel) override;

    virtual void applyJT(const sofa::core::MechanicalParams* mparams,
                 const sofa::type::vector<InDataVecDeriv*>& dataVecOutForce,
                 const sofa::type::vector<const OutDataVecDeriv*>& dataVecInForce) override;

    virtual void applyJT(const sofa::core::ConstraintParams* cparams,
             const sofa::type::vector<InDataMatrixDeriv*>& dataVecOutConst,
             const sofa::type::vector<const OutDataMatrixDeriv*>& dataVecInConst) override;
    
    virtual void applyDJT(const sofa::core::MechanicalParams* mparams,
              sofa::core::MultiVecDerivId inForce,
              sofa::core::ConstMultiVecDerivId outForce) override;

protected:
    /// cache input values from apply() for use in applyJT()
    sofa::type::vector<sofa::type::vector<SReal>> m_values;
    std::size_t m_totalSize{0};
};

} // namespace sofadiff