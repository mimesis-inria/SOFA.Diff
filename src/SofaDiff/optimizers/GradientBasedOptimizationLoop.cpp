#include <SofaDiff/optimizers/GradientBasedOptimizationLoop.h>


namespace sofadiff
{

void GradientBasedOptimizationLoop::init()
{
    OptimizationLoop::init();
    m_differentiableAnimationLoop = dynamic_cast<DifferentiableAnimationLoop *>(m_animationLoop);
}


void GradientBasedOptimizationLoop::step(const core::ExecParams *params, SReal dt)
{
    OptimizationLoop::step(params, dt);

    for (int i = 0; i < m_totalTimesteps.getValue(); i++)
        m_differentiableAnimationLoop->stepAdjoint(params, dt);

    // TODO: go back to t=t_max?
}


}