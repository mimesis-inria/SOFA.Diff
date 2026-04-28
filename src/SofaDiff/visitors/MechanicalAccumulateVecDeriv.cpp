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
#include <SofaDiff/visitors/MechanicalAccumulateVecDeriv.h>


namespace sofadiff
{

MechanicalAccumulateVecDeriv::MechanicalAccumulateVecDeriv(const core::MechanicalParams* params, const core::MultiVecDerivId& vecId)
: MechanicalVisitor(params),
m_vecId(vecId)
{}

void MechanicalAccumulateVecDeriv::bwdMechanicalMapping(simulation::Node * node, core::BaseMapping *map)
{
    // TODO: remove timing with begin() and end()?
    const ctime_t t0 = begin(node, map);
    map->applyJT(mparams, m_vecId, m_vecId);
    end(node, map, t0);
}

bool MechanicalAccumulateVecDeriv::stopAtMechanicalMapping(simulation::Node*, core::BaseMapping*)
{
    // This visitor must go through all mechanical mappings, even if isMechanical flag is disabled
    return false;
}

bool MechanicalAccumulateVecDeriv::isThreadSafe() const
{
    return false; // TODO: false?
}

const char* MechanicalAccumulateVecDeriv::getClassName() const
{
    return "MechanicalAccumulateVecDeriv";
}
std::string MechanicalAccumulateVecDeriv::getInfos() const
{
    std::string name="["+m_vecId.getName()+"]";
    return name;
}

}
