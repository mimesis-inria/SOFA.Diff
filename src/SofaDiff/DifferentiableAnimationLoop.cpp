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

#include <SofaDiff/DifferentiableAnimationLoop.h>
#include <SofaDiff/visitors/StoreStateVisitor.h>
#include <SofaDiff/visitors/RetrieveStateVisitor.h>
#include <SofaDiff/adjoints/AdjointSolver.h>
#include <SofaDiff/utils.h>

#include <sofa/core/ObjectFactory.h>


namespace sofadiff
{
using namespace sofa::core;

void registerDifferentiableAnimationLoop(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Controller that performs gradient descent.").add< DifferentiableAnimationLoop >());
}

DifferentiableAnimationLoop::DifferentiableAnimationLoop():
    d_totalTimesteps(initData(&d_totalTimesteps, 0, "timesteps", "Number of timesteps for the simulation to be complete. Default is 0, meaning indefinite simulation.")),
    m_currentTimestep(0)
{}

void DifferentiableAnimationLoop::init()
{
    DefaultAnimationLoop::init();
    LinearSolverAccessor::init();

    BaseContext * rootContext = this->getContext()->getRootContext();
    m_startPositionId = newVecId<V_COORD>(rootContext, "startPosition", this->getClassName());
    m_startVelocityId = newVecId<V_DERIV>(rootContext, "startVelocity", this->getClassName());
}

void DifferentiableAnimationLoop::step(const ExecParams* params, const SReal dt)
{
    if (!isStepAllowed())
        return;

    if (m_currentTimestep == 0)
    {
        StoreStateVisitor visitor(execparams::defaultInstance(), m_startPositionId, m_startVelocityId);
        visitor.execute(this->getContext());
    }

    DefaultAnimationLoop::step(params, dt);
    m_currentTimestep++;
}

void DifferentiableAnimationLoop::stepAdjoint(const ExecParams* params, SReal dt)
{
    if (!isStepAdjointAllowed())
        return;

    // Get adjoints and losses
    std::vector<AdjointSolver*> adjoints;
    this->getContext()->get<AdjointSolver>(&adjoints, BaseContext::SearchDown);
    std::vector<LossState*> lossStates;
    this->getContext()->get<LossState>(&lossStates, BaseContext::SearchDown);

    // Reset the gradients
    for (const auto adjoint : adjoints)
        adjoint->resetGradients(params);

    // Initialize backpropagation
    for (const auto loss : lossStates)
        loss->d_gradient.setValue(std::vector {1, type::Vec1d(1.0)});

    // Perform backpropagation
    for (const auto adjoint : adjoints)
        adjoint->solve(params, dt, vec_id::write_access::position, vec_id::write_access::velocity);

    // Reset simulation
    resetSimulation();
}

void DifferentiableAnimationLoop::resetSimulation()
{
    if (!isResetSimulationAllowed())
        return;

    m_currentTimestep = 0;
    RetrieveStateVisitor visitor(execparams::defaultInstance(), m_startPositionId, m_startVelocityId);
    visitor.execute(this->getContext());
}

bool DifferentiableAnimationLoop::isStepAllowed() const
{
    if (this->d_componentState.getValue() != ComponentState::Valid)
        return false;
    const int totalTimesteps = this->getTotalTimesteps();
    if (totalTimesteps > 0 && this->getCurrentTimestep() >= totalTimesteps)
        return false;
    return true;
}

bool DifferentiableAnimationLoop::isStepAdjointAllowed() const
{
    if (this->d_componentState.getValue() != ComponentState::Valid)
        return false;
    const int totalTimesteps = this->getTotalTimesteps();
    if (totalTimesteps > 0 && this->getCurrentTimestep() != totalTimesteps)
        return false;
    if (this->getCurrentTimestep() == 0)
        return false;
    return true;
}

bool DifferentiableAnimationLoop::isResetSimulationAllowed() const
{
    if (this->d_componentState.getValue() != ComponentState::Valid)
        return false;
    if (this->getCurrentTimestep() == 0)
        return false;
    return true;
}

}
