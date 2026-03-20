#include <SofaDiff/optimizers/OptimizationLoop.h>
#include <SofaDiff/DifferentiableAnimationLoop.h>
#include <SofaDiff/utils.h>
#include <SofaDiff/visitors/StoreStateVisitor.h>
#include <SofaDiff/visitors/RetrieveStateVisitor.h>


namespace sofadiff
{
using namespace sofa::core::objectmodel;

OptimizationLoop::OptimizationLoop():
    l_simulationLoop(initLink("simulationLoop", "Link to a SimulationLoop")),
    d_maxOptimizationSteps(initData(&d_maxOptimizationSteps, "maxOptimizationSteps", "Maximum number of optimization steps for the optimization to be complete. Default is 0, meaning indefinite optimization.")),
    d_maxSimulationSteps(initData(&d_maxSimulationSteps, 1, "maxSimulationSteps", "Maximum number of simulation steps to optimize. 0 means as many as required for the simulation to be complete. Default is 1.")),
    m_readyToUpdateParameters(false),
    m_currentOptimizationStep(0),
    m_lowestLossValue(std::numeric_limits<SReal>::max()),
    m_startingTime(0)
{}

void OptimizationLoop::init()
{
    Inherit1::init();

    this->initializeSimulationLink();

    BaseContext * rootContext = this->getContext()->getRootContext();
    m_startingPositionId = newVecId<V_COORD>(rootContext, "startingPosition", this->getClassName());
    m_startingVelocityId = newVecId<V_DERIV>(rootContext, "startingVelocity", this->getClassName());

    if (this->d_componentState.getValue() != ComponentState::Invalid)
        this->d_componentState.setValue(ComponentState::Valid);
}

void OptimizationLoop::bwdInit()
{
    auto * node = dynamic_cast<simulation::Node *>(this->getContext());
    node->animationManager.set(this);

    this->setStartingState();  // By default, start the optimization from the initial state
}

void OptimizationLoop::step(const ExecParams *params, const SReal dt)
{
    if (!isStepAllowed())
        return;

    this->updateParameters();
    this->computeLoss(params, dt);
    this->processSimulation(params, dt);
    this->setParametersNextValue();
    m_readyToUpdateParameters = true; // TODO: somehow put that in setParametersNextValue()

    m_currentOptimizationStep++;

    // Note:
    // 1. The parameters' next value are computed here, before anything can alter the state of the simulation.
    // 2. The actual update is delayed to the beginning of the next optimization step, to keep the consistency between
    //    the displayed value of the parameters and the displayed solution (computed with said value).

    // This way of doing things is not very compatible with multiple OptimizationLoops: switching from one to another
    // applies the update of the previous optimizer, and no reset occurs at any point. One way to fix the first point
    // would be to have each optimizer have its "own" nextValue for each parameter. For the second point, some
    // optimizers should be reset when "switched out" (e.g. gradient descent, because the gradient is no longer valid),
    // but others should not (e.g. grid search, because the different steps are independent). Unless they optimize
    // different parameters... But this is not a feature provided for now.

    // Also, the computing of the parameters' next value should not change the state of the simulation, so that the
    // step() ends showing the final state, which is typically the most interesting one. If that is not possible (e.g.
    // for computing the gradient in the dynamic case), then the final state should be stored before the computations,
    // and retrieved afterward.
}

void OptimizationLoop::setBestParameters(const ExecParams *params, const SReal dt)
{
    std::vector<BaseParameter*> parameters;
    this->getContext()->get<BaseParameter>(&parameters, BaseContext::SearchDown);
    for (auto * parameter : parameters)
        parameter->retrieveBestValue();
    this->computeLoss(params, dt);
}

void OptimizationLoop::resetOptimization()
{
    if (!isResetOptimizationAllowed())
        return;

    m_currentOptimizationStep = 0;
    m_readyToUpdateParameters = false;
    m_lowestLossValue = std::numeric_limits<SReal>::max();
}

void OptimizationLoop::setStartingState()
{
    if (!isSetStartingStateAllowed())
        return;

    // Store the current state for later retrieval
    m_startingTime = this->getTime();
    StoreStateVisitor visitor(execparams::defaultInstance(), m_startingPositionId, m_startingVelocityId);
    visitor.execute(this->getContext());
}

bool OptimizationLoop::isStepAllowed() const
{
    if (this->d_componentState.getValue() != ComponentState::Valid)
        return false;
    if (this->getMaxOptimizationSteps() > 0 && this->getCurrentOptimizationStep() >= this->getMaxOptimizationSteps())
        return false;
    return true;
}

bool OptimizationLoop::isSetBestParametersAllowed() const
{
    if (this->d_componentState.getValue() != ComponentState::Valid)
        return false;
    if (this->getCurrentOptimizationStep() <= 0)
        return false;
    return true;
}

bool OptimizationLoop::isResetOptimizationAllowed() const
{
    if (this->d_componentState.getValue() != ComponentState::Valid)
        return false;
    if (this->getCurrentOptimizationStep() == 0)
        return false;
    return true;
}

bool OptimizationLoop::isSetStartingStateAllowed() const
{
    if (this->d_componentState.getValue() != ComponentState::Valid)
        return false;
    if (this->getCurrentOptimizationStep() != 0)
        return false;
    return true;
}

void OptimizationLoop::updateParameters()
{
    if (m_readyToUpdateParameters)
    {
        std::vector<BaseParameter*> parameters;
        this->getContext()->get<BaseParameter>(&parameters, BaseContext::SearchDown);
        for (auto * parameter : parameters)
            parameter->updateValue();
        m_readyToUpdateParameters = false;
    }
}

void OptimizationLoop::computeLoss(const ExecParams *params, const SReal dt)
{
    dynamic_cast<simulation::Node*>(this->l_node.get())->setTime(m_startingTime);
    RetrieveStateVisitor visitor(execparams::defaultInstance(), m_startingPositionId, m_startingVelocityId);
    visitor.execute(this->getContext());

    if (auto * differentiableLoop = dynamic_cast<DifferentiableAnimationLoop*>(l_simulationLoop.get()))
        differentiableLoop->resetAdjoint(params);  // TODO: more like `resetForward()`

    const int simulationSteps = getSimulationSteps();
    for (int i = 0; i < simulationSteps; i++)
        l_simulationLoop->step(params, dt);
}

void OptimizationLoop::processSimulation(const ExecParams *params, SReal dt)
{
    SOFA_UNUSED(params);
    SOFA_UNUSED(dt);

    SReal loss = 0;
    std::vector<LossState*> losses;
    this->getContext()->get<LossState>(&losses, BaseContext::SearchDown);
    for (const auto lossState : losses)
        loss += lossState->d_value.getValue()[0][0];

    if (loss < m_lowestLossValue)
    {
        m_lowestLossValue = loss;
        std::vector<BaseParameter*> parameters;
        this->getContext()->get<BaseParameter>(&parameters, BaseContext::SearchDown);
        for (auto * parameter : parameters)
            parameter->storeBestValue();
    }
}

void OptimizationLoop::initializeSimulationLink()
{
    if (l_simulationLoop.get())
        return;

    const auto * ctx = this->getContext();
    l_simulationLoop.set(ctx->get<simulation::DefaultAnimationLoop>(ctx->getTags(), BaseContext::SearchDown));
    if (l_simulationLoop)
        return;

    msg_warning() << "A SimulationLoop is required by this component but has not been found. It will be created automatically";
    const auto simulationLoop = objectmodel::New<simulation::DefaultAnimationLoop>();
    simulationLoop->setName(this->getContext()->getNameHelper().resolveName(simulationLoop->getClassName(), ComponentNameHelper::Convention::xml));
    simulationLoop->d_computeBoundingBox.setValue(false);
    this->getContext()->addObject(simulationLoop);
    simulationLoop->init();
    l_simulationLoop.set(simulationLoop);
}

int OptimizationLoop::getSimulationSteps()
{
    int simulationSteps = d_maxSimulationSteps.getValue();
    const auto * differentiableLoop = dynamic_cast<DifferentiableAnimationLoop*>(l_simulationLoop.get());
    if (differentiableLoop && differentiableLoop->getMaxSimulationSteps() > 0)
    {   // The simulation has a max number of steps (ideally all simulation loops would have this option)
        const int remainingSteps = differentiableLoop->getMaxSimulationSteps() - differentiableLoop->getCurrentSimulationStep();
        simulationSteps = simulationSteps > 0 ? std::min(remainingSteps, simulationSteps) : remainingSteps;
    }
    if (simulationSteps <= 0)
    {
        // TODO: This error could be corrected at runtime, but how to handle that?
        msg_error() << "No simulation step to perform. Either you have not provided a positive number of simulationSteps for this optimization, or the starting state is the final state";
        this->d_componentState.setValue(ComponentState::Invalid);
    }
    return simulationSteps;
}

}
