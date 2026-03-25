#include <SofaDiff/bindings/Binding_GradientBasedOptimizationLoop.h>
#include <SofaDiff/bindings/Binding_OptimizationLoop.h>
#include <SofaDiff/bindings/Binding_Parameter.h>

#include <pybind11/pybind11.h>


namespace py
{
using namespace pybind11;
}

namespace sofapython3
{

PYBIND11_MODULE(SofaDiff, m)
{
    moduleAddParameter(m);
    moduleAddOptimizationLoop(m);
    moduleAddGradientBasedOptimizationLoop(m);
}

}