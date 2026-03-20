#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/optimizers/OptimizationLoop.h>

#include <sofa/core/ExecParams.h>


namespace sofadiff
{

class SOFA_SOFADIFF_API GridSearchOptimizationLoop: public OptimizationLoop
{
public:
    SOFA_CLASS(GridSearchOptimizationLoop, OptimizationLoop);

    void init() override;
    void resetOptimization() override;
    void setParametersNextValue() override;

protected:
    void setParameters(int iteration);

    int m_gridSize;
};

}
