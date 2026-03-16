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

    void init() override;
    void bwdInit() override;

    void step(const ExecParams *params, SReal dt) override;
    void resetOptimization();
    void setStartingState();

    bool isStepAllowed() const;
    bool isResetOptimizationAllowed() const;
    bool isSetStartingStateAllowed() const;

    int getMaxSimulationSteps() const      { return d_maxSimulationSteps.getValue(); }
    int getMaxOptimizationSteps() const    { return d_maxOptimizationSteps.getValue(); }
    int getCurrentOptimizationStep() const { return m_currentOptimizationStep; }

    void setMaxSimulationSteps(const int maxSteps)   { d_maxSimulationSteps.setValue(maxSteps); }
    void setMaxOptimizationSteps(const int maxSteps) { d_maxOptimizationSteps.setValue(maxSteps); }

private:
    virtual void computeParametersNextValue(const ExecParams *params, SReal dt) = 0;
    virtual void initializeSimulationLink();
    int getSimulationSteps();

public:
    SingleLink<OptimizationLoop, BaseAnimationLoop, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_simulationLoop;
    Data<int> d_maxOptimizationSteps;
    Data<int> d_maxSimulationSteps;

protected:
    bool m_readyToUpdateParameters;
    int m_currentOptimizationStep;

    SReal m_startingTime;
    MultiVecCoordId m_startingPositionId;
    MultiVecDerivId m_startingVelocityId;
};

}
