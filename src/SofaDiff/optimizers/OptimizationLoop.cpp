#include <SofaDiff/optimizers/OptimizationLoop.h>
#include <SofaDiff/DifferentiableAnimationLoop.h>
#include <SofaDiff/utils.h>
#include <SofaDiff/visitors/StoreStateVisitor.h>
#include <SofaDiff/visitors/RetrieveStateVisitor.h>


namespace sofadiff
{

OptimizationLoop::OptimizationLoop():
    l_simulationLoop(initLink("simulationLoop", "Link to a SimulationLoop")),
    d_maxOptimizationSteps(initData(&d_maxOptimizationSteps, "maxOptimizationSteps", "Maximum number of optimization steps for the optimization to be complete. Default is 0, meaning indefinite optimization.")),
    d_maxSimulationSteps(initData(&d_maxSimulationSteps, 1, "maxSimulationSteps", "Maximum number of simulation steps to optimize. 0 means as many as required for the simulation to be complete. Default is 1.")),
    m_currentOptimizationStep(0),
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

    // Process the results from the previous optimization step and update the parameters
    if (m_currentOptimizationStep > 0)
        updateParameters(params, dt);

    // Prepare for the update of the loss with the new parameters
    dynamic_cast<simulation::Node*>(this->l_node.get())->setTime(m_startingTime);
    RetrieveStateVisitor visitor(execparams::defaultInstance(), m_startingPositionId, m_startingVelocityId);
    visitor.execute(this->getContext());

    if (auto * differentiableLoop = dynamic_cast<DifferentiableAnimationLoop*>(l_simulationLoop.get()))
        differentiableLoop->resetSimulation();

    // Update the loss
    const int simulationSteps = getSimulationSteps();
    for (int i = 0; i < simulationSteps; i++)
        l_simulationLoop->step(params, dt);

    // Note: I do things in this order (first update parameters, then compute loss) so that the parameters and solution
    // accessible to the user are consistent, instead of having the solution coming from the previous parameters, and
    // the current parameters being for the next optimization step.
    // The downside is that the user could mess with the simulation between here and the update of the parameters... So
    // we should reset the optimization (m_currentStep = 0) if that were to happen. But that seems impossible.
    // A better solution would be to process the results here, while the user cannot intervene, and store the parameters
    // to be used in the next optimization step, and show both the parameters that were used, and the future parameters.
    // However, in the differentiable + dynamic case, processing the results means doing the adjoint solve, which brings
    // the system back to the starting state; thus showing useless stuff to the user.
    // A solution for this issue could be to store the final state and retrieve it once the adjoint is done. This seems
    // like a good idea.

    m_currentOptimizationStep++;
}

void OptimizationLoop::resetOptimization()
{
    m_currentOptimizationStep = 0;
}

void OptimizationLoop::setStartingState()
{
    // Store the current state for later retrieval
    m_startingTime = this->getTime();
    StoreStateVisitor visitor(execparams::defaultInstance(), m_startingPositionId, m_startingVelocityId);
    visitor.execute(this->getContext());
}

bool OptimizationLoop::isStepAllowed() const
{
    if (this->d_componentState.getValue() != ComponentState::Valid)
        return false;
    const int totalIterations = this->getMaxOptimizationSteps();
    if (totalIterations > 0 && this->getCurrentOptimizationStep() >= totalIterations)
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

void OptimizationLoop::initializeSimulationLink()
{
    if (l_simulationLoop.get())
        return;

    const auto * ctx = this->getContext();

    l_simulationLoop.set(ctx->get<simulation::DefaultAnimationLoop>(ctx->getTags(), BaseContext::SearchDown));
    if (l_simulationLoop)
        return;

    l_simulationLoop.set(ctx->get<DifferentiableAnimationLoop>(ctx->getTags(), BaseContext::SearchDown));
    if (l_simulationLoop)
        return;

    msg_warning() << "A SimulationLoop is required by this component but has not been found. It will be created automatically";
    const auto simulationLoop = core::objectmodel::New<simulation::DefaultAnimationLoop>();
    simulationLoop->setName(this->getContext()->getNameHelper().resolveName(simulationLoop->getClassName(), core::ComponentNameHelper::Convention::xml));
    this->getContext()->addObject(simulationLoop);
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
