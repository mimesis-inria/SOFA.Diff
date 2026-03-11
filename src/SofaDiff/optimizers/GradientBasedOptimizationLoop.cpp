#include <SofaDiff/optimizers/GradientBasedOptimizationLoop.h>


namespace sofadiff
{

void GradientBasedOptimizationLoop::init()
{
    OptimizationLoop::init();
    m_differentiableAnimationLoop = dynamic_cast<DifferentiableAnimationLoop *>(m_animationLoop);
}

bool GradientBasedOptimizationLoop::isUpdateReady()
{
    // if (m_differentiableAnimationLoop->getTimestepIndex() == 0 && m_differentiableAnimationLoop->getTimestepTotal() > 0)
    // {
    //     return true;
    // }
    // return false;
    return true;
}

void GradientBasedOptimizationLoop::updateParameters()
{
    applyGradient();
    // m_differentiableAnimationLoop->resetDifferentiableMode();
}

void GradientBasedOptimizationLoop::step(const core::ExecParams *params, SReal dt)
{
    // m_differentiableAnimationLoop->resetDifferentiableMode(); // In case the user played with the DifferentiableAnimationLoop
    // m_differentiableAnimationLoop->setDifferentiableMode(true);

    OptimizationLoop::step(params, dt);
    for (int i = 0; i < m_totalTimesteps.getValue(); i++)
        m_differentiableAnimationLoop->stepAdjoint(params, dt);

    // m_differentiableAnimationLoop->setDifferentiableMode(false);

    // TODO: go back to t=t_max?
}


}