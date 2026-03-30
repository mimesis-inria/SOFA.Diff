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

private:
    void _initialize() override;
    void _updateParameters() override;

    void setParameters(int iteration);
    int m_gridSize;
};

}
