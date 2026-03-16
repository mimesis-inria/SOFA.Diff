#include <SofaDiff/optimizers/GradientBasedOptimizationLoop.h>


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

void GradientBasedOptimizationLoop::computeParametersNextValue(const ExecParams *params, const SReal dt)
{
    const auto simulationLoop = dynamic_cast<DifferentiableAnimationLoop*>(l_simulationLoop.get());

    if (!simulationLoop)
    {
        msg_error() << "Requires a DifferentiableAnimationLoop";
        this->d_componentState.setValue(ComponentState::Invalid);
        return;
    }

    simulationLoop->stepAdjoint(params, dt);
}

}