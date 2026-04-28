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
#include <SofaDiff/visitors/AdjointResetVisitor.h>
#include <SofaDiff/adjoints/AdjointSolver.h>

#include <sofa/simulation/Node.h>
#include <sofa/core/objectmodel/BaseContext.h>


namespace sofadiff
{
using namespace sofa::core;


AdjointResetVisitor::AdjointResetVisitor(const ExecParams *params): Visitor(params)
{}


simulation::Visitor::Result AdjointResetVisitor::processNodeTopDown(simulation::Node* node)
{
    if (! node->solver.empty())
    {
        // TODO: better way to get the adjoint?
        std::vector<AdjointSolver *> adjoints;
        const auto* ctx = node->getContext();
        ctx->get<AdjointSolver> (&adjoints, objectmodel::BaseContext::Local); // TODO: correct search direction?
        if (adjoints.empty())
        {
            msg_error("No adjoint solver found");
            return RESULT_PRUNE;
        }
        if (adjoints.size() > 1)
        {
            msg_warning("Multiple adjoint solvers found, picking the first one"); // TODO: print name of the picked adjoint
        }
        AdjointSolver * adjoint = adjoints[0];
        adjoint->resetGradients(params); // TODO: task scheduling like SolveVisitor?
        return RESULT_PRUNE;
    }

    // TODO: What is the equivalent of what follows for the adjoint?
    // if (m_computeForceIsolatedInteractionForceFields)
    // {
    //     for_each(this, node, node->interactionForceField, &SolveVisitor::fwdInteractionForceField);
    // }
    return RESULT_CONTINUE;
}


void AdjointResetVisitor::processNodeBottomUp(simulation::Node* /*node*/)
{

}

}
