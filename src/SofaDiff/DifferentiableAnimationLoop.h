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

#include <SofaDiff/config.h>
#include <SofaDiff/ParameterizedForceField.h>
#include <SofaDiff/LossState.h>

#include <sofa/simulation/DefaultAnimationLoop.h>
#include <sofa/core/behavior/LinearSolverAccessor.h>
#include <sofa/core/behavior/MultiVec.h>


namespace sofadiff
{
enum SolverDirection {NONE, FORWARD, BACKWARD};

class SOFA_SOFADIFF_API DifferentiableAnimationLoop:
    public simulation::DefaultAnimationLoop,
    public core::behavior::LinearSolverAccessor
{
public:
    SOFA_CLASS2(DifferentiableAnimationLoop, sofa::simulation::DefaultAnimationLoop, core::behavior::LinearSolverAccessor);

    void init() override;
    void step(const core::ExecParams* params, SReal dt) override;
    void stepAdjoint(const core::ExecParams* params, SReal dt);

protected:
    std::vector<LossState *> m_lossStates;
    void setLossGradient(SReal value);

    void storeState(const core::ExecParams* params);
    void retrieveState(const core::ExecParams* params);

private:
    std::vector<core::MultiVecCoordId> m_positionStorage;
    std::vector<core::MultiVecDerivId> m_velocityStorage;
    unsigned int m_index; // index of the current time step
    SolverDirection m_solverDirection;
    unsigned int m_totalTimesteps; // number of consecutive time steps
};

}