#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/optimizers/GradientBasedOptimizationLoop.h>


namespace sofadiff
{

class SOFA_SOFADIFF_API GradientDescentOptimizationLoop: public GradientBasedOptimizationLoop
{
public:
    SOFA_CLASS(GradientDescentOptimizationLoop, GradientBasedOptimizationLoop);

    void computeParametersNextValue(const ExecParams *params, SReal dt) override;
};

}
