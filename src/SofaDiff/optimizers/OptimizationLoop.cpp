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
    l_parameters(initLink("parameters", "Link to the parameters to optimize")),
    d_maxOptimizationSteps(initData(&d_maxOptimizationSteps, "maxOptimizationSteps", "Maximum number of optimization steps for the optimization to be complete. Default is 0, meaning indefinite optimization.")),
    d_maxSimulationSteps(initData(&d_maxSimulationSteps, 1, "maxSimulationSteps", "Maximum number of simulation steps to optimize. 0 means as many as required for the simulation to be complete. Default is 1.")),
    d_startWithUpdate(initData(&d_startWithUpdate, false, "startWithUpdate", "Whether to start with an update of the parameters (true), or from the given current parameters (false). Default is false.")),
    m_readyToApplyUpdate(false),
    m_currentOptimizationStep(0),
    m_lowestLossValue(std::numeric_limits<SReal>::max()),
    m_startingTime(0)
{}

void OptimizationLoop::init()
{
    Inherit1::init();
    this->allocate();
    this->initialize();

    if (this->d_componentState.getValue() != ComponentState::Invalid)
        this->d_componentState.setValue(ComponentState::Valid);
}

void OptimizationLoop::allocate()
{
    this->initializeSimulationLink();

    this->initializeParametersLink();

    this->enterParameterGroup();
    for (const auto& parameter : l_parameters)
    {
        parameter->newData("nextValue");
        parameter->newData("bestValue");
    }

    BaseContext * rootContext = this->getContext()->getRootContext();
    m_startingPositionId = newVecId<V_COORD>(rootContext, "startingPosition", this->getClassName());
    m_startingVelocityId = newVecId<V_DERIV>(rootContext, "startingVelocity", this->getClassName());

    this->_allocate();
}

void OptimizationLoop::initialize()
{
    m_currentOptimizationStep = 0;
    m_readyToApplyUpdate = false;
    m_lowestLossValue = std::numeric_limits<SReal>::max();

    this->_initialize();

    if (d_startWithUpdate.getValue())
        this->updateParameters();
}

void OptimizationLoop::bwdInit()
{
    auto * node = dynamic_cast<simulation::Node *>(this->getContext());
    node->animationManager.set(this);

    this->setStartingState();  // By default, start the optimization from the initial state

    this->leaveParameterGroup();
}

void OptimizationLoop::step(const ExecParams *params, const SReal dt)
{
    if (!isStepAllowed())
        return;

    this->enterParameterGroup();

    this->applyUpdate();
    this->computeLoss(params, dt);
    this->processSimulation(params, dt);
    this->updateParameters();
    m_currentOptimizationStep++;

    this->leaveParameterGroup();

    // Note:
    // 1. The parameters' next value are computed here, before anything can alter the state of the simulation.
    // 2. The actual update is delayed to the beginning of the next optimization step, to keep the consistency between
    //    the displayed value of the parameters and the displayed solution (computed with said value).

    // Also, the computing of the parameters' next value should not change the state of the simulation, so that the
    // step() ends showing the final state, which is typically the most interesting one. If that is not possible (e.g.
    // for computing the gradient in the dynamic case), then the final state should be stored before the computations,
    // and retrieved afterward.
}

void OptimizationLoop::retrieveBestParameters(const ExecParams *params, const SReal dt)
{
    if (!isRetrieveBestParametersAllowed())
        return;

    this->enterParameterGroup();

    for (auto & parameter : l_parameters)
        parameter->setDataFrom("value", "bestValue");
    this->computeLoss(params, dt);

    this->leaveParameterGroup();
}

void OptimizationLoop::resetOptimization()
{
    if (!isResetOptimizationAllowed())
        return;

    this->enterParameterGroup();
    this->initialize();
    this->leaveParameterGroup();
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

bool OptimizationLoop::isRetrieveBestParametersAllowed() const
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

void OptimizationLoop::applyUpdate()
{
    if (m_readyToApplyUpdate)
    {
        for (auto & parameter : l_parameters)
            parameter->setDataFrom("value", "nextValue");
        m_readyToApplyUpdate = false;
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

void OptimizationLoop::processSimulation(const ExecParams *params, const SReal dt)
{
    SReal loss = 0;
    std::vector<LossState*> losses;
    this->getContext()->get<LossState>(&losses, BaseContext::SearchDown);
    for (const auto lossState : losses)
        loss += lossState->d_value.getValue()[0][0];

    if (loss < m_lowestLossValue)
    {
        m_lowestLossValue = loss;
        for (auto & parameter : l_parameters)
            parameter->setDataFrom("bestValue", "value");
    }

    this->_processSimulation(params, dt);
}

void OptimizationLoop::updateParameters()
{
    this->_updateParameters();
    m_readyToApplyUpdate = true;
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

void OptimizationLoop::initializeParametersLink()
{
    const auto n = l_parameters.size();

    if (n == 0)
    {
        msg_warning() << "No parameter provided. You must specify the parameters to be optimized with 'parameters=[...]'";
        this->d_componentState.setValue(ComponentState::Invalid);
        return;
    }

    for (unsigned int i = 0; i < n; i++)
    {
        if (!l_parameters.get(i))
        {
            msg_warning() << "Invalid parameter: " << l_parameters.getLinkedPath(i);
            this->d_componentState.setValue(ComponentState::Invalid);
        }
    }
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

void OptimizationLoop::enterParameterGroup() const
{
    for (auto & parameter : l_parameters)
        parameter->enterGroup(this->getName());
}

void OptimizationLoop::leaveParameterGroup() const
{
    for (auto & parameter : l_parameters)
        parameter->leaveGroup();
}

}
