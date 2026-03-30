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
    auto object = py_shared_ptr(new GradientBasedOptimizationLoop_Trampoline());
    object->f_listening.setValue(true);

    if (!args.empty())
    {
        msg_warning("GradientBasedOptimizationLoop") << "Positional arguments were passed to the constructor, but only keyword arguments are handled";
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

void GradientBasedOptimizationLoop_Trampoline::_allocate()
{
    py::gil_scoped_acquire gil;
    const py::function py_override = py::get_override(this, "allocate");
    if (!py_override)
        return;  // throw std::runtime_error("allocate() not implemented in Python subclass");
    (void) py_override();  // The cast tells the IDE that discarding the return value is intentional
}

void GradientBasedOptimizationLoop_Trampoline::_initialize()
{
    py::gil_scoped_acquire gil;
    const py::function py_override = py::get_override(this, "initialize");
    if (!py_override)
        return;  // throw std::runtime_error("initialize() not implemented in Python subclass");
    (void) py_override();  // The cast tells the IDE that discarding the return value is intentional
}

void GradientBasedOptimizationLoop_Trampoline::_updateParameters()
{
    py::gil_scoped_acquire gil;
    const py::function py_override = py::get_override(this, "update_parameters");
    if (!py_override)
        throw std::runtime_error("update_parameters() not implemented in Python subclass");
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

    py::class_<GradientBasedOptimizationLoop,
               OptimizationLoop,
               GradientBasedOptimizationLoop_Trampoline,
               py_shared_ptr<GradientBasedOptimizationLoop>>
        f(m, pyclass_name.c_str(), py::dynamic_attr(), py::multiple_inheritance());

    f.def(py::init(&GradientBasedOptimizationLoop_Trampoline::create));
    // The rest of the interface is inherited from OptimizationLoop

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
