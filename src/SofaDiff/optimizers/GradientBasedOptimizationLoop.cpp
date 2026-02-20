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

    // TODO: make as many stepAdjoint() as required to go back to t=0
    for (int i = 0; i < 60; i++)
        m_differentiableAnimationLoop->stepAdjoint(params, dt);

    // TODO: go back to t=t_max?
}


}