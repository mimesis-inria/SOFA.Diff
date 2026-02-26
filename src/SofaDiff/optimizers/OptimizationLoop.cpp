#include <SofaDiff/optimizers/OptimizationLoop.h>
#include <SofaDiff/ControlLoop.h>

#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>


namespace sofadiff
{

OptimizationLoop::OptimizationLoop():
    m_totalTimesteps(initData(&m_totalTimesteps, "timesteps", "Number of time steps for the optimization")),
    m_animationLoop(nullptr)
{

}


void OptimizationLoop::init()
{
    Inherit1::init();

    const auto* ctx = this->getContext();

    std::vector<BaseAnimationLoop *> loops;
    ctx->get<BaseAnimationLoop> (&loops, core::objectmodel::BaseContext::SearchRoot);
    for (const auto loop : loops)
    {
        if (loop == this || dynamic_cast<ControlLoop*>(loop) || dynamic_cast<OptimizationLoop*>(loop))
        {
            continue;
        }
        m_animationLoop = loop;
    }

    ctx->get<BaseParameter> (&m_parameters, core::objectmodel::BaseContext::SearchRoot);
}


// Using this method was the idea of Paul Baksic
void OptimizationLoop::bwdInit()
{
    auto * node = dynamic_cast<simulation::Node *>(this->getContext());
    node->animationManager.set(this);
}


void OptimizationLoop::step(const core::ExecParams *params, SReal dt)
{
    updateParameters();

    for (int i = 0; i < m_totalTimesteps.getValue(); i++)
        m_animationLoop->step(params, dt);
}


SReal OptimizationLoop::getHyperparameter(const BaseParameter *parameter, const std::string &hyperparameterName) const
{
    const auto * baseData = parameter->findData(hyperparameterName);
    if (baseData == nullptr)
    {
        msg_error() << "Parameter " << parameter->getName() << " requires a data " << hyperparameterName;
        return 0.0;
    }

    const auto * data = dynamic_cast<const Data<SReal>*>(baseData);
    if (data == nullptr)
    {
        // Unlikely to trigger since the string value of the hyperparameter is converted to a double with std::stod()
        msg_error() << "Hyperparameter " << hyperparameterName << " of parameter " << parameter->getName() << " should be scalar";
        return 0.0;
    }

    return data->getValue();
}


}
