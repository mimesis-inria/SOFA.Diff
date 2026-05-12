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
#include <sofa/diff/adjoints/AdjointSolver.h>

#include <sofa/core/behavior/LinearSolverAccessor.h>


namespace sofadiff
{
using namespace sofa::core;


class SOFA_SOFADIFF_API StaticAdjointSolver: public AdjointSolver, public behavior::LinearSolverAccessor
{
public:
    SOFA_CLASS2(StaticAdjointSolver, AdjointSolver, core::behavior::LinearSolverAccessor);

    void init() override;
    void resetGradients(const ExecParams *) override;
    void solve(const ExecParams* /*params*/, SReal /*dt*/, MultiVecCoordId /*xResult*/, MultiVecDerivId /*vResult*/) override;

protected:
    MultiVecDerivId m_positionGradientId;
    MultiVecDerivId m_forceGradientId;

    MultiVecDerivId & getLossGradientId() override { return m_positionGradientId; }

private:
    void solveForPhysicalGradient();
};

}