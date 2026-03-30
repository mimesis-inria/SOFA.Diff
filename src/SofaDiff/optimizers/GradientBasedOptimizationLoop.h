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

private:
    void initializeSimulationLink() override;  // Make sure the simulation is differentiable
    void _processSimulation(const ExecParams *params, SReal dt) override;  // Compute the gradient

    void _updateParameters() override = 0;  // Just a reminder of the abstractness of the class

};

}
