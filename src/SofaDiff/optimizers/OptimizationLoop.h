#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/Parameter.h>

#include <sofa/core/behavior/BaseAnimationLoop.h>


namespace sofadiff
{

class SOFA_SOFADIFF_API OptimizationLoop: public core::behavior::BaseAnimationLoop
{
public:
    SOFA_ABSTRACT_CLASS(OptimizationLoop, core::behavior::BaseAnimationLoop);

    OptimizationLoop();

// Inherited methods
    void init() override;
    void bwdInit() override;
    void step(const core::ExecParams *params, SReal dt) override;

// New methods
    void resetOptimization();

    bool isStepAllowed() const;
    bool isResetOptimizationAllowed() const;

    int getTotalIterations() const { return d_totalIterations.getValue(); }
    int getCurrentIteration() const { return m_currentIteration; }

    void setTotalIterations(const int totalIterations) { d_totalIterations.setValue(totalIterations); }

protected:
    virtual void updateParameters() = 0;

// New attributes
public:
    Data<int> d_totalIterations;

protected:
    int m_currentIteration;
};

}
