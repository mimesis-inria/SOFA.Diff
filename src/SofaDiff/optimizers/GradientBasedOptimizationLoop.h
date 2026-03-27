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

    void _processSimulation(const ExecParams *params, SReal dt) override;
    void _updateParameters() override = 0;  // Just a reminder of the abstractness of the class
private:
    void initializeSimulationLink() override;
};

}
