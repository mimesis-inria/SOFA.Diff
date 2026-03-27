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

    void step(const ExecParams *params, SReal dt) final;
    void setBestParameters(const ExecParams *params, SReal dt);
    virtual void resetOptimization();
    void setStartingState();

    bool isStepAllowed() const;
    bool isSetBestParametersAllowed() const;
    bool isResetOptimizationAllowed() const;
    bool isSetStartingStateAllowed() const;

    int getMaxSimulationSteps() const      { return d_maxSimulationSteps.getValue(); }
    int getMaxOptimizationSteps() const    { return d_maxOptimizationSteps.getValue(); }
    int getCurrentOptimizationStep() const { return m_currentOptimizationStep; }

    void setMaxSimulationSteps(const int maxSteps)   { d_maxSimulationSteps.setValue(maxSteps); }
    void setMaxOptimizationSteps(const int maxSteps) { d_maxOptimizationSteps.setValue(maxSteps); }

    void createParameterData(BaseParameter & parameter, const std::string & name) const
        { parameter.newDataInGroup(name, this->getName()); }
    void setParameterData(BaseParameter & parameter, const std::string & name, const type::vector<SReal> & vector) const
        { parameter.setDataInGroupFromVector(name, this->getName(), vector); }
    const type::vector<SReal> & getParameterData(const BaseParameter & parameter, const std::string & name) const
        { return parameter.getVectorFromDataInGroup(name, this->getName()); }

protected:
    virtual void processSimulation(const ExecParams *params, SReal dt);

private:
    void updateParameters();
    void computeLoss(const ExecParams *params, SReal dt);
    virtual void setParametersNextValue() = 0;
    virtual void initializeSimulationLink();
    void initializeParametersLink();
    int getSimulationSteps();

public:
    SingleLink<OptimizationLoop, BaseAnimationLoop, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_simulationLoop;
    MultiLink<OptimizationLoop, BaseParameter, BaseLink::FLAG_STOREPATH> l_parameters;
    Data<int> d_maxOptimizationSteps;
    Data<int> d_maxSimulationSteps;

protected:
    bool m_readyToUpdateParameters;
    int m_currentOptimizationStep;
    SReal m_lowestLossValue;

    SReal m_startingTime;
    MultiVecCoordId m_startingPositionId;
    MultiVecDerivId m_startingVelocityId;
};

}
