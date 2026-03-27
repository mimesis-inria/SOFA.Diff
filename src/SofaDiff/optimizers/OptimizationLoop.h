#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/Parameter.h>

#include <sofa/core/MultiVecId.h>
#include <sofa/core/behavior/BaseAnimationLoop.h>


namespace sofadiff
{
using namespace sofa::core;

class SOFA_SOFADIFF_API OptimizationLoop: public behavior::BaseAnimationLoop
{
public:
    SOFA_ABSTRACT_CLASS(OptimizationLoop, behavior::BaseAnimationLoop);

    OptimizationLoop();
    SingleLink<OptimizationLoop, BaseAnimationLoop, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_simulationLoop;
    MultiLink<OptimizationLoop, BaseParameter, BaseLink::FLAG_STOREPATH> l_parameters;
    Data<int> d_maxOptimizationSteps;
    Data<int> d_maxSimulationSteps;

    void init() final;
    void bwdInit() final;

    void step(const ExecParams *params, SReal dt) final;
    void retrieveBestParameters(const ExecParams *params, SReal dt);
    void resetOptimization();
    void setStartingState();

    bool isStepAllowed() const;
    bool isRetrieveBestParametersAllowed() const;
    bool isResetOptimizationAllowed() const;
    bool isSetStartingStateAllowed() const;

    int getMaxSimulationSteps() const      { return d_maxSimulationSteps.getValue(); }
    int getMaxOptimizationSteps() const    { return d_maxOptimizationSteps.getValue(); }
    int getCurrentOptimizationStep() const { return m_currentOptimizationStep; }

    void setMaxSimulationSteps(const int maxSteps)   { d_maxSimulationSteps.setValue(maxSteps); }
    void setMaxOptimizationSteps(const int maxSteps) { d_maxOptimizationSteps.setValue(maxSteps); }

// Should be private, but bindings require public
    virtual void _allocate() {}
    virtual void _initialize() {}
    virtual void _processSimulation(const ExecParams * /*params*/, SReal /*dt*/) {}
    virtual void _updateParameters() = 0;

public: // TODO: Temporary
    virtual void initializeSimulationLink();

    void initializeParametersLink();
    int getSimulationSteps();

    void enterParameterGroup();
    void leaveParameterGroup();

    void allocate();
    void initialize();
    void applyUpdate();
    void computeLoss(const ExecParams *params, SReal dt);
    void processSimulation(const ExecParams *params, SReal dt);
    void updateParameters();

protected:
    bool m_readyToApplyUpdate;
    int m_currentOptimizationStep;
    SReal m_lowestLossValue;

    SReal m_startingTime;
    MultiVecCoordId m_startingPositionId;
    MultiVecDerivId m_startingVelocityId;
};

}
