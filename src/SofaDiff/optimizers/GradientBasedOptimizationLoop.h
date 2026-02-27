#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/optimizers/OptimizationLoop.h>

#include <SofaDiff/DifferentiableAnimationLoop.h>


namespace sofadiff
{

class SOFA_SOFADIFF_API GradientBasedOptimizationLoop: public OptimizationLoop
{
public:
    SOFA_ABSTRACT_CLASS(GradientBasedOptimizationLoop, OptimizationLoop);

    void init() override;
    void step(const core::ExecParams *params, SReal dt) override;
    bool isUpdateReady() override;
    void updateParameters() final;

    virtual void applyGradient() = 0;

protected:
    DifferentiableAnimationLoop * m_differentiableAnimationLoop;
};

}
