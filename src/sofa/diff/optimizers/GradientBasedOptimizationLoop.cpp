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
#include <sofa/diff/optimizers/GradientBasedOptimizationLoop.h>


namespace sofadiff
{
using namespace sofa::core::objectmodel;

void GradientBasedOptimizationLoop::initializeSimulationLink()
{
    if (l_simulationLoop.get())
        return;

    const auto * ctx = this->getContext();
    l_simulationLoop.set(ctx->get<DifferentiableAnimationLoop>(ctx->getTags(), BaseContext::SearchDown));
    if (l_simulationLoop)
        return;

    msg_warning() << "A DifferentiableSimulationLoop is required by this component but has not been found. It will be created automatically";
    const auto simulationLoop = objectmodel::New<DifferentiableAnimationLoop>();
    simulationLoop->setName(this->getContext()->getNameHelper().resolveName(simulationLoop->getClassName(), ComponentNameHelper::Convention::xml));
    this->getContext()->addObject(simulationLoop);
    l_simulationLoop.set(simulationLoop);
}

void GradientBasedOptimizationLoop::_processSimulation(const ExecParams *params, const SReal dt)
{
    const auto simulationLoop = dynamic_cast<DifferentiableAnimationLoop*>(l_simulationLoop.get());
    if (!simulationLoop)
    {
        msg_error() << "Requires a DifferentiableAnimationLoop";
        this->d_componentState.setValue(ComponentState::Invalid);
        return;
    }
    // TODO: simulationLoop->resetAdjoint(); to reset the gradients
    simulationLoop->stepAdjoint(params, dt);
}

}