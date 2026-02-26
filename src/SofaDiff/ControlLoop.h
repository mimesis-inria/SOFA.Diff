#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/Parameter.h>
#include <SofaDiff/optimizers/OptimizationLoop.h>

#include <sofa/core/behavior/BaseAnimationLoop.h>


namespace sofadiff
{

class SOFA_SOFADIFF_API ControlLoop: public core::behavior::BaseAnimationLoop
{
public:
    SOFA_CLASS(ControlLoop, core::behavior::BaseAnimationLoop);

    ControlLoop();

    void init() override;
    void bwdInit() override;

    void step(const core::ExecParams *params, SReal dt) override;

protected:
    Data<int> m_maxOptimizationIterations;
    BaseAnimationLoop * m_animationLoop;
    BaseAnimationLoop * m_optimizationLoop;
};

}
