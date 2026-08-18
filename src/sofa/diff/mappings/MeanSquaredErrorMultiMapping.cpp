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
#include <sofa/diff/config.h>
#include <sofa/diff/mappings/MeanSquaredErrorMultiMapping.h>

#include <sofa/core/ObjectFactory.h>

namespace sofadiff
{

using namespace defaulttype;

void MeanSquaredErrorMultiMapping::apply(const sofa::core::MechanicalParams*,
const sofa::type::vector<OutDataVecCoord*>& dataVecOutPos,
const sofa::type::vector<const InDataVecCoord*>& dataVecInPos)
{
    if (dataVecOutPos.empty())
        return;

    auto result = sofa::helper::getWriteAccessor(*dataVecOutPos[0]);

    m_values.resize(dataVecInPos.size());
    m_totalSize = 0;

    SReal sum = 0.0;

    for (std::size_t p = 0; p < dataVecInPos.size(); ++p)
    {
        const auto values = sofa::helper::getReadAccessor(*dataVecInPos[p]);
        m_values[p].resize(values.size());
        m_totalSize += values.size();

        for (std::size_t i = 0; i < values.size(); ++i)
        {
            const SReal v = values[i][0];
            m_values[p][i] = v;
            sum += v * v;
        }
    }

    if (m_totalSize == 0)
        result[0][0] = 0.0;
    else
        result[0][0] = sum / static_cast<SReal>(m_totalSize);
}

void MeanSquaredErrorMultiMapping::applyJ(const sofa::core::MechanicalParams*,
                                          const sofa::type::vector<OutDataVecDeriv*>& dataVecOutVel,
                                          const sofa::type::vector<const InDataVecDeriv*>& dataVecInVel)
{

}

void MeanSquaredErrorMultiMapping::applyJT(const sofa::core::MechanicalParams*,
                                           const sofa::type::vector<InDataVecDeriv*>& dataVecOutForce,
                                           const sofa::type::vector<const OutDataVecDeriv*>& dataVecInForce)
{
    if (dataVecInForce.empty() || m_totalSize == 0)
        return;

    const auto values = sofa::helper::getReadAccessor(*dataVecInForce[0]);
    const SReal factor = 2.0 * values[0][0] / static_cast<SReal>(m_totalSize);

    const std::size_t nParents = std::min(dataVecOutForce.size(), m_values.size());

    for (std::size_t p = 0; p < nParents; ++p)
    {
        auto result = sofa::helper::getWriteAccessor(*dataVecOutForce[p]);
        const std::size_t n = std::min(result.size(), m_values[p].size());

        for (std::size_t i = 0; i < n; ++i)
        {
            result[i][0] += factor * m_values[p][i];
        }
    }
}

void MeanSquaredErrorMultiMapping::applyJT(const sofa::core::ConstraintParams*,
                                           const sofa::type::vector<InDataMatrixDeriv*>& dataVecOutConst,
                                           const sofa::type::vector<const OutDataMatrixDeriv*>& dataVecInConst)
{
    
}

void MeanSquaredErrorMultiMapping::applyDJT(const sofa::core::MechanicalParams*,
                                            sofa::core::MultiVecDerivId,
                                            sofa::core::ConstMultiVecDerivId)
{
    // No geometric stiffness contribution for this mapping.
}

void registerMeanSquaredErrorMultiMapping(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData("Mapping multiple Vec1d inputs to the mean of all their squares.")
            .add<MeanSquaredErrorMultiMapping>());
}

} // namespace sofadiff