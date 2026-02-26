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
    m_differentiableAnimationLoop->setDifferentiableMode(true);

    OptimizationLoop::step(params, dt);
    for (int i = 0; i < m_totalTimesteps.getValue(); i++)
        m_differentiableAnimationLoop->stepAdjoint(params, dt);

    m_differentiableAnimationLoop->setDifferentiableMode(false);

    // TODO: go back to t=t_max?
}


}