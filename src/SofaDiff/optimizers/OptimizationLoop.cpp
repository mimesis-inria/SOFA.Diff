#include <SofaDiff/optimizers/OptimizationLoop.h>

#include <sofa/simulation/Node.h>


namespace sofadiff
{

void OptimizationLoop::init()
{
    Inherit1::init();

    const auto* ctx = this->getContext();

    std::vector<BaseAnimationLoop *> loops;
    ctx->get<BaseAnimationLoop> (&loops, core::objectmodel::BaseContext::SearchRoot);
    for (const auto loop : loops)
    {
        if (loop != this) // TODO: more robust detection (in case there are more than 2 loops)
        {
            m_animationLoop = loop;
            break;
        }
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

    // TODO: make as many steps as required to compute the loss
    for (int i = 0; i < 60; i++)
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