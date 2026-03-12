#include <SofaDiff/optimizers/OptimizationLoop.h>
#include <SofaDiff/DifferentiableAnimationLoop.h>

#include <sofa/simulation/DefaultAnimationLoop.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>


namespace sofadiff
{

OptimizationLoop::OptimizationLoop():
    d_totalIterations(initData(&d_totalIterations, "iterations", "Number of iterations for the optimization to be complete. Default is 0, meaning indefinite optimization.")),
    m_currentIteration(0)
{}

void OptimizationLoop::init()
{
    Inherit1::init();
    this->d_componentState.setValue(ComponentState::Valid);
}

// Using this method was the idea of Paul Baksic
void OptimizationLoop::bwdInit()
{
    auto * node = dynamic_cast<simulation::Node *>(this->getContext());
    node->animationManager.set(this);
}

void OptimizationLoop::step(const core::ExecParams *params, const SReal dt)
{
    if (!isStepAllowed())
        return;

    // Get simulation loops
    std::vector<simulation::DefaultAnimationLoop*> defaultSimulators;
    std::vector<DifferentiableAnimationLoop*> differentiableSimulators;
    this->getContext()->get<simulation::DefaultAnimationLoop>(&defaultSimulators, BaseContext::SearchDown);
    this->getContext()->get<DifferentiableAnimationLoop>(&differentiableSimulators, BaseContext::SearchDown);

    if (m_currentIteration > 0)
    {
        // Reset simulation
        for (const auto simulator : differentiableSimulators)
            simulator->resetSimulation();

        // Update parameters
        this->updateParameters();
    }

    // Compute loss
    for (const auto simulator : differentiableSimulators)
        for (int i = 0; i < simulator->getTotalTimesteps(); ++i)
            simulator->step(params, dt);

    m_currentIteration++;
}

void OptimizationLoop::resetOptimization()
{
    m_currentIteration = 0;
}

bool OptimizationLoop::isStepAllowed() const
{
    if (this->d_componentState.getValue() != ComponentState::Valid)
        return false;
    const int totalIterations = this->getTotalIterations();
    if (totalIterations > 0 && this->getCurrentIteration() >= totalIterations)
        return false;
    return true;
}

bool OptimizationLoop::isResetOptimizationAllowed() const
{
    if (this->d_componentState.getValue() != ComponentState::Valid)
        return false;
    if (this->getCurrentIteration() == 0)
        return false;
    return true;
}


}
