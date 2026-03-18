#include <SofaDiff/bindings/Binding_GradientBasedOptimizationLoop.h>
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
    moduleAddGradientBasedOptimizationLoop(m);
}

}