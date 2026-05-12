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
#include <sofa/diff/ParameterizedForceField.h>
#include <sofa/diff/LossState.h>

#include <sofa/simulation/DefaultAnimationLoop.h>
#include <sofa/core/behavior/LinearSolverAccessor.h>
#include <sofa/core/behavior/MultiVec.h>


namespace sofadiff
{
using namespace sofa::core;

class SOFA_SOFADIFF_API DifferentiableAnimationLoop:
    public simulation::DefaultAnimationLoop,
    public behavior::LinearSolverAccessor
{
public:
    SOFA_CLASS2(DifferentiableAnimationLoop, sofa::simulation::DefaultAnimationLoop, behavior::LinearSolverAccessor);
    DifferentiableAnimationLoop();

// Inherited methods
    void init() override;
    void step(const ExecParams* params, SReal dt) override;

// New methods
    void stepAdjoint(const ExecParams* params, SReal dt);
    void resetAdjoint(const ExecParams* params);

    bool isStepAllowed() const;
    bool isStepAdjointAllowed() const;
    bool isResetAdjointAllowed() const;

    int getMaxSimulationSteps() const { return d_maxSimulationSteps.getValue(); }
    int getCurrentSimulationStep() const { return m_currentSimulationStep; }

    void setMaxSimulationSteps(const int maxSteps) { d_maxSimulationSteps.setValue(maxSteps); }

// New attributes
    Data<int> d_maxSimulationSteps;

protected:
    int m_currentSimulationStep;
};

}