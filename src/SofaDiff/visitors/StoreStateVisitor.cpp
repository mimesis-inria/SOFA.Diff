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
#include <SofaDiff/visitors/StoreStateVisitor.h>

#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/VectorOperations.h>


namespace sofadiff
{
using namespace sofa::core;


StoreStateVisitor::StoreStateVisitor(const ExecParams *params, const MultiVecCoordId& positionId, const MultiVecDerivId& velocityId):
    Visitor(params),
    m_positionId(positionId),
    m_velocityId(velocityId)
{}


simulation::Visitor::Result StoreStateVisitor::processNodeTopDown(simulation::Node* node)
{
    if (!node->mechanicalState)
    {
        return RESULT_CONTINUE;
    }

    simulation::common::VectorOperations vop(params, node->getContext());
    vop.v_eq(m_positionId, vec_id::read_access::position);
    vop.v_eq(m_velocityId, vec_id::read_access::velocity);
    return RESULT_CONTINUE;
}


void StoreStateVisitor::processNodeBottomUp(simulation::Node* /*node*/)
{

}

}
