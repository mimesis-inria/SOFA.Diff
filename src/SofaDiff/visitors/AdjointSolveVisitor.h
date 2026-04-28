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
#include <sofa/simulation/Visitor.h>


namespace sofadiff
{
using namespace sofa::core;


class SOFA_SOFADIFF_API AdjointSolveVisitor: public simulation::Visitor
{
public:
    AdjointSolveVisitor(const ExecParams* params, SReal _dt, const MultiVecCoordId& X, const MultiVecDerivId &V);

    Result processNodeTopDown(simulation::Node* node) override;
    void processNodeBottomUp(simulation::Node* /*node*/) override;

    /// Specify whether this action can be parallelized.
    bool isThreadSafe() const override { return false; } // TODO: I have no idea of the correct return value here

    /// Return a category name for this action.
    /// Only used for debugging / profiling purposes
    const char* getCategoryName() const override { return "behavior update position"; } // TODO: No idea
    const char* getClassName() const override { return "AdjointSolveVisitor"; }

protected:
    SReal dt;
    MultiVecCoordId x;
    MultiVecDerivId v;
};
}