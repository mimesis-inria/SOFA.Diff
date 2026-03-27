#include <pybind11/stl.h>

#include <SofaDiff/bindings/Binding_OptimizationLoop.h>

#include <SofaPython3/Sofa/Core/Binding_Base.h>


namespace sofapython3
{
namespace py { using namespace pybind11; }
using namespace py::literals;


py_shared_ptr<OptimizationLoop_Trampoline> OptimizationLoop_Trampoline::create(const py::args& args, const py::kwargs& kwargs)
{
    auto object = py_shared_ptr(new OptimizationLoop_Trampoline());
    object->f_listening.setValue(true);

    if (!args.empty())
    {
        msg_warning("OptimizationLoop") << "Positional arguments were passed to the constructor, but only keyword arguments are handled";
    }

    const py::object pyObject = py::cast(object);
    for (auto [pyKey, pyValue] : kwargs)
    {
        auto key = py::cast<std::string>(pyKey);
        const auto value = py::reinterpret_borrow<py::object>(pyValue);
        setattr(pyObject, key.c_str(), value);
    }

    return object;
}

void OptimizationLoop_Trampoline::_allocate()
{
    PYBIND11_OVERRIDE(void, OptimizationLoop, _allocate, );
}

void OptimizationLoop_Trampoline::_initialize()
{
    PYBIND11_OVERRIDE(void, OptimizationLoop, _initialize, );
}

void OptimizationLoop_Trampoline::_processSimulation(const ExecParams * params, SReal dt)
{
    PYBIND11_OVERRIDE(void, OptimizationLoop, _processSimulation, params, dt);
}

void OptimizationLoop_Trampoline::_updateParameters()
{
    PYBIND11_OVERRIDE_PURE(void, OptimizationLoop, _updateParameters, );
    // // I want to use a different name for the Python method, so the macros won't do here
    // py::gil_scoped_acquire gil;
    // const py::function py_override = py::get_override(this, "update_parameters");
    // if (!py_override)
    //     throw std::runtime_error("update_parameters() not implemented in Python subclass");
    // (void) py_override();  // The cast tells the IDE that discarding the return value is intentional
}

std::string OptimizationLoop_Trampoline::getClassName() const
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
        return "trampoline_OptimizationLoop_cast_err";
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error in getClassName: " << e.what() << std::endl;
        return "trampoline_OptimizationLoop_err";
    }
}


void moduleAddOptimizationLoop(const pybind11::module& m)
{
    const auto pyclass_name = std::string("OptimizationLoop");

    py::class_<OptimizationLoop, objectmodel::BaseObject, OptimizationLoop_Trampoline, py_shared_ptr<OptimizationLoop>>
        f(m, pyclass_name.c_str(), py::dynamic_attr(), py::multiple_inheritance());

    f.def(py::init(&OptimizationLoop_Trampoline::create));
    f.def("_allocate", &OptimizationLoop::_allocate);
    f.def("_initialize", &OptimizationLoop::_initialize);
    f.def("_processSimulation", &OptimizationLoop::_processSimulation);
    f.def("_updateParameters", &OptimizationLoop::_updateParameters);
    f.def("updateParameters", &OptimizationLoop::updateParameters);

    // [](OptimizationLoop&) {
    //     throw std::runtime_error("compute_next_value() must be overridden in a subclass");
    // }); // This def() exposes the method that has to be overridden by the derived class, for documentation purposes

    f.def_property_readonly("parameters", [](const OptimizationLoop& self) {
        std::vector<BaseParameter*> parameters;
        for (const auto& parameter : self.l_parameters)
            parameters.push_back(parameter);
        return parameters;
    });

    PythonFactory::registerType<OptimizationLoop>(
        [](objectmodel::Base* object) -> py::object
        {
            auto* sp = dynamic_cast<OptimizationLoop*>(object);
            if (!sp) return py::none();
            try
            {
                return py::cast(sp);
            }
            catch (const std::exception& e)
            {
                return py::cast(dynamic_cast<OptimizationLoop*>(object));
            }
        }
    );
}

}
