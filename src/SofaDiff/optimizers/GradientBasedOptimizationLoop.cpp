#include <SofaDiff/optimizers/GradientBasedOptimizationLoop.h>


namespace sofadiff
{

void GradientBasedOptimizationLoop::step(const core::ExecParams *params, SReal dt)
{
    if (m_currentIteration > 0)
    {
        std::vector<DifferentiableAnimationLoop*> differentiableSimulators;
        this->getContext()->get<DifferentiableAnimationLoop>(&differentiableSimulators, BaseContext::SearchDown);
        for (auto * simulator : differentiableSimulators)
        {
            simulator->stepAdjoint(params, dt);
        }
    }

    OptimizationLoop::step(params, dt);
}


}