#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/optimizers/OptimizationLoop.h>

#include <SofaPython3/PythonFactory.h>
#include <SofaPython3/Sofa/Core/Binding_Base.h>
#include <sofa/core/ExecParams.h>


namespace sofapython3
{
using namespace sofadiff;
namespace py { using namespace pybind11; }

class OptimizationLoop_Trampoline : public OptimizationLoop
{
public:
    SOFA_CLASS(OptimizationLoop_Trampoline, OptimizationLoop); // Useful?

    /* Inherit the constructors */
    using OptimizationLoop::OptimizationLoop;
    static py_shared_ptr<OptimizationLoop_Trampoline> create(const py::args& args, const py::kwargs& kwargs);

    void init() override;
    void bwdInit() override;

    void resetOptimization() override;
    void computeParametersNextValue(const ExecParams *params, SReal dt) final;

    std::string getClassName() const override;
};

void moduleAddOptimizationLoop(const pybind11::module& m);

}
