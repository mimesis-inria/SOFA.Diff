#include <SofaDiff/ControlLoop.h>
#include <SofaDiff/DifferentiableAnimationLoop.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>


namespace sofadiff
{

void registerControlLoop(core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Control loop based on an optimization loop.").add< ControlLoop >());
}

ControlLoop::ControlLoop():
    m_maxOptimizationIterations(initData(&m_maxOptimizationIterations, "maxOptimizationIterations", "Maximum number of iterations of the optimizer")),
    m_animationLoop(nullptr),
    m_optimizationLoop(nullptr)
{}

void ControlLoop::init()
{
    BaseAnimationLoop::init();

    const auto* ctx = this->getContext();

    std::vector<BaseAnimationLoop *> loops;
    ctx->get<BaseAnimationLoop> (&loops, BaseContext::SearchRoot);
    for (const auto loop : loops)
    {
        if (loop == this)
        {
            continue;
        }
        if (dynamic_cast<ControlLoop*>(loop))
        {
            msg_error("Multiple ControlLoops detected: behavior undefined.");
            continue;
        }

        if (dynamic_cast<OptimizationLoop*>(loop))
        {
            m_optimizationLoop = loop;
        }
        else
        {
            m_animationLoop = loop;
        }
    }
}

// Using this method was the idea of Paul Baksic
void ControlLoop::bwdInit()
{
    auto * node = dynamic_cast<simulation::Node *>(this->getContext());
    node->animationManager.set(this);
}

void ControlLoop::step(const core::ExecParams *params, const SReal dt)
{
    // Find optimal parameters
    for (int i = 0; i < m_maxOptimizationIterations.getValue(); i++)
        m_optimizationLoop->step(params, dt);
    // Use optimal parameters
    m_animationLoop->step(params, dt);
}

}
