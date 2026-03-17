#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/optimizers/GradientBasedOptimizationLoop.h>

#include <SofaPython3/PythonFactory.h>


namespace sofapython3
{
class GradientBasedOptimizationLoop_Trampoline : public sofadiff::GradientBasedOptimizationLoop
{
public:
    SOFA_CLASS(GradientBasedOptimizationLoop_Trampoline, sofadiff::GradientBasedOptimizationLoop);

    GradientBasedOptimizationLoop_Trampoline();
    ~GradientBasedOptimizationLoop_Trampoline() override;

    void init() override;
    std::string getClassName() const override;

    void computeParametersNextValue(const core::ExecParams *params, SReal dt) override;
    virtual std::vector<SReal> getNextValue(const std::vector<SReal>& value, const std::vector<SReal>& gradient, SReal learningRate);
};

void initGradientBasedOptimizationLoop(const pybind11::module& m);

}
