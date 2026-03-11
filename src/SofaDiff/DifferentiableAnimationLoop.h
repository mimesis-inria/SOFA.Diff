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
class SOFA_SOFADIFF_API DifferentiableAnimationLoop:
    public simulation::DefaultAnimationLoop,
    public core::behavior::LinearSolverAccessor
{
public:
    SOFA_CLASS2(DifferentiableAnimationLoop, sofa::simulation::DefaultAnimationLoop, core::behavior::LinearSolverAccessor);
    DifferentiableAnimationLoop();

// Inherited methods
    void init() override;
    void step(const core::ExecParams* params, SReal dt) override;

// New methods
    void stepAdjoint(const core::ExecParams* params, SReal dt);
    void resetSimulation();

    bool isStepAllowed() const;
    bool isStepAdjointAllowed() const;
    bool isResetSimulationAllowed() const;

    int getTotalTimesteps() const { return d_totalTimesteps.getValue(); }
    int getCurrentTimestep() const { return m_currentTimestep; }

    void setTotalTimesteps(const int totalTimesteps) { d_totalTimesteps.setValue(totalTimesteps); }

// New attributes
    Data<int> d_totalTimesteps;

protected:
    int m_currentTimestep;

    core::MultiVecCoordId m_startPositionId;
    core::MultiVecDerivId m_startVelocityId;
};

}