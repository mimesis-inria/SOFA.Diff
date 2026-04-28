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
#include <SofaDiff/DifferentiableAnimationLoop.h>
#include <SofaDiff/adjoints/AdjointSolver.h>

#include <sofa/core/ObjectFactory.h>


namespace sofadiff
{
using namespace sofa::core;
using namespace sofa::core::objectmodel;

void registerDifferentiableAnimationLoop(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Controller that performs gradient descent.").add< DifferentiableAnimationLoop >());
}

DifferentiableAnimationLoop::DifferentiableAnimationLoop():
    d_maxSimulationSteps(initData(&d_maxSimulationSteps, 0, "timesteps", "Number of timesteps for the simulation to be complete. Default is 0, meaning indefinite simulation.")),
    m_currentSimulationStep(0)
{}

void DifferentiableAnimationLoop::init()
{
    DefaultAnimationLoop::init();
    LinearSolverAccessor::init();
}

void DifferentiableAnimationLoop::step(const ExecParams* params, const SReal dt)
{
    if (!isStepAllowed())
        return;

    DefaultAnimationLoop::step(params, dt);
    m_currentSimulationStep++;
}

void DifferentiableAnimationLoop::stepAdjoint(const ExecParams* params, const SReal dt)
{
    if (!isStepAdjointAllowed())
        return;

    // Initialize backpropagation
    std::vector<LossState*> lossStates;
    this->getContext()->get<LossState>(&lossStates, BaseContext::SearchDown);
    for (const auto loss : lossStates)
        loss->d_gradient.setValue(std::vector {1, type::Vec1d(1.0)});

    // Perform backpropagation
    std::vector<AdjointSolver*> adjoints;
    this->getContext()->get<AdjointSolver>(&adjoints, BaseContext::SearchDown);
    for (const auto adjoint : adjoints)
    {
        adjoint->resetGradients(params);  // Only for static
        adjoint->solve(params, dt, vec_id::write_access::position, vec_id::write_access::velocity);
    }
}

void DifferentiableAnimationLoop::resetAdjoint(const ExecParams* params)
{
    if (!isResetAdjointAllowed())
        return;

    std::vector<AdjointSolver*> adjoints;
    this->getContext()->get<AdjointSolver>(&adjoints, BaseContext::SearchDown);
    for (const auto adjoint : adjoints)
        adjoint->resetGradients(params);
    m_currentSimulationStep = 0;
}

bool DifferentiableAnimationLoop::isStepAllowed() const
{
    if (this->d_componentState.getValue() != ComponentState::Valid)
        return false;
    if (this->getMaxSimulationSteps() > 0 && this->getCurrentSimulationStep() >= this->getMaxSimulationSteps())
        return false;
    return true;
}

bool DifferentiableAnimationLoop::isStepAdjointAllowed() const
{
    if (this->d_componentState.getValue() != ComponentState::Valid)
        return false;
    if (this->getCurrentSimulationStep() <= 0)
        return false;
    return true;
}

bool DifferentiableAnimationLoop::isResetAdjointAllowed() const
{
    if (this->d_componentState.getValue() != ComponentState::Valid)
        return false;
    return true;
}

}
