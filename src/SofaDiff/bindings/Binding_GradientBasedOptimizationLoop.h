#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/optimizers/GradientBasedOptimizationLoop.h>

#include <SofaPython3/PythonFactory.h>
#include <SofaPython3/Sofa/Core/Binding_Base.h>


namespace sofapython3
{
namespace py { using namespace pybind11; }

class GradientBasedOptimizationLoop_Trampoline: public sofadiff::GradientBasedOptimizationLoop
{
public:
    SOFA_CLASS(GradientBasedOptimizationLoop_Trampoline, sofadiff::GradientBasedOptimizationLoop);

    using GradientBasedOptimizationLoop::GradientBasedOptimizationLoop;  /* Inherit constructors */
    static py_shared_ptr<GradientBasedOptimizationLoop_Trampoline> create(const py::args& args, const py::kwargs& kwargs);

    void init() override;
    void bwdInit() override;

    void resetOptimization() override;
    void computeParametersNextValue(const core::ExecParams *params, SReal dt) override;

    std::string getClassName() const override;
};

void moduleAddGradientBasedOptimizationLoop(const pybind11::module& m);

}
