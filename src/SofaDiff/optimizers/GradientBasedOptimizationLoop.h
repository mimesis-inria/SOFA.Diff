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

    void processSimulation(const ExecParams *params, SReal dt) override;
    void setParametersNextValue() override = 0;
private:
    void initializeSimulationLink() override;
};

}
