#include <pybind11/stl.h>

#include <SofaDiff/bindings/Binding_GradientBasedOptimizationLoop.h>

#include <SofaPython3/Sofa/Core/Binding_Base.h>


namespace sofapython3
{
namespace py { using namespace pybind11; }
using namespace py::literals;
using namespace sofa::core;
using namespace sofadiff;


py_shared_ptr<GradientBasedOptimizationLoop_Trampoline> GradientBasedOptimizationLoop_Trampoline::create(const py::args& args, const py::kwargs& kwargs)
{
    auto ff = py_shared_ptr(new GradientBasedOptimizationLoop_Trampoline());
    ff->f_listening.setValue(true);

    if (args.size() == 1) ff->setName(py::cast<std::string>(args[0]));

    py::object cc = py::cast(ff);
    for (auto [pyKey, pyValue] : kwargs)
    {
        auto key = py::cast<std::string>(pyKey);
        auto value = py::reinterpret_borrow<py::object>(pyValue);
        if (key == "name")
        {
            ff->setName(py::cast<std::string>(value));
        }
    }

    return ff;
}

void GradientBasedOptimizationLoop_Trampoline::init()
{
    PYBIND11_OVERRIDE(void, GradientBasedOptimizationLoop, init, );
}

void GradientBasedOptimizationLoop_Trampoline::bwdInit()
{
    PYBIND11_OVERRIDE(void, GradientBasedOptimizationLoop, bwdInit, );
}

void GradientBasedOptimizationLoop_Trampoline::resetOptimization()
{
    PYBIND11_OVERRIDE(void, GradientBasedOptimizationLoop, resetOptimization, );
}

void GradientBasedOptimizationLoop_Trampoline::computeParametersNextValue(const ExecParams *params, const SReal dt)
{
    GradientBasedOptimizationLoop::computeParametersNextValue(params, dt);

    py::gil_scoped_acquire gil;
    const py::function py_override = py::get_override(this, "compute_next_value");
    if (!py_override)
        throw std::runtime_error("compute_next_value() not implemented in Python subclass");
    (void) py_override();  // The cast tells the IDE that discarding the return value is intentional
}

std::string GradientBasedOptimizationLoop_Trampoline::getClassName() const
{
    py::gil_scoped_acquire gil_acquire;

    try
    {
        const py::object self = py::cast(this); // will throw if no Python object exists
        auto name = py::str(self.get_type().attr("__name__"));
        return name;
    }
    catch (const py::cast_error&)
    {
        return "trampoline_GradientBasedOptimizationLoop_cast_err";
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error in getClassName: " << e.what() << std::endl;
        return "trampoline_GradientBasedOptimizationLoop_err";
    }
}

void moduleAddGradientBasedOptimizationLoop(const pybind11::module& m)
{
    const auto pyclass_name = std::string("GradientBasedOptimizationLoop");

    py::class_<sofadiff::GradientBasedOptimizationLoop,
               sofadiff::OptimizationLoop,
               GradientBasedOptimizationLoop_Trampoline,
               py_shared_ptr<sofadiff::GradientBasedOptimizationLoop>>
        f(m, pyclass_name.c_str(), py::dynamic_attr(), py::multiple_inheritance());

    f.def(py::init(&GradientBasedOptimizationLoop_Trampoline::create));
    f.def("init", &GradientBasedOptimizationLoop::init);
    f.def("bwdInit", &GradientBasedOptimizationLoop::bwdInit);
    f.def("resetOptimization", &GradientBasedOptimizationLoop::resetOptimization);
    f.def("compute_next_value", [](GradientBasedOptimizationLoop&) {
        throw std::runtime_error("compute_next_value() must be overridden in a subclass");
    }); // This def() exposes the method that has to be overridden by the derived class, for documentation purposes

    PythonFactory::registerType<GradientBasedOptimizationLoop>(
        [](objectmodel::Base* object) -> py::object
        {
            auto* sp = dynamic_cast<GradientBasedOptimizationLoop*>(object);
            if (!sp) return py::none();
            try
            {
                return py::cast(sp);
            }
            catch (const std::exception& e)
            {
                return py::cast(dynamic_cast<GradientBasedOptimizationLoop*>(object));
            }
        }
    );
}

}
