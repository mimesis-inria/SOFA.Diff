#include <pybind11/stl.h>

#include <SofaDiff/bindings/Binding_GradientBasedOptimizationLoop.h>

#include <SofaPython3/Sofa/Core/Binding_Base.h>


namespace sofapython3
{
/// Makes an alias for the pybind11 namespace to increase readability.
namespace py { using namespace pybind11; }
using namespace py::literals;


GradientBasedOptimizationLoop_Trampoline::GradientBasedOptimizationLoop_Trampoline():
    GradientBasedOptimizationLoop()
{
}

GradientBasedOptimizationLoop_Trampoline::~GradientBasedOptimizationLoop_Trampoline()
{
}

void GradientBasedOptimizationLoop_Trampoline::init()
{
    GradientBasedOptimizationLoop::init();

    py::gil_scoped_acquire gil_acquire;
    if (const py::function override = py::get_overload(static_cast<const GradientBasedOptimizationLoop*>(this), "init"))
    {
        try
        {
            override();
        }
        catch (const py::error_already_set& e)
        {
            std::cerr << "Python error in init override:\n" << e.what() << std::endl;
            throw;
        }
    }
}


void GradientBasedOptimizationLoop_Trampoline::computeParametersNextValue(const core::ExecParams *params, const SReal dt)
{
    GradientBasedOptimizationLoop::computeParametersNextValue(params, dt);
    this->computeNextValue();
}

void GradientBasedOptimizationLoop_Trampoline::computeNextValue()
{
    py::gil_scoped_acquire gil_acquire;
    // PYBIND11_OVERLOAD(std::vector<SReal>, GradientBasedOptimizationLoop, getNextValue, value, gradient, learningRate);
    if (const py::function override = py::get_overload(static_cast<const GradientBasedOptimizationLoop*>(this), "compute_next_value"))
    {
        try
        {
            override();
        }
        catch (const py::error_already_set& e)
        {
            std::cerr << "Python error in get_next_value override:\n" << e.what() << std::endl;
            throw;
        }
    }
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
               core::objectmodel::BaseObject,
               GradientBasedOptimizationLoop_Trampoline,
               py_shared_ptr<sofadiff::GradientBasedOptimizationLoop>>
        f(m, pyclass_name.c_str(), py::dynamic_attr(), py::multiple_inheritance());

    f.def(py::init(
        [](const py::args& args, py::kwargs& /*kwargs*/)
        {
            auto ff = py_shared_ptr(new GradientBasedOptimizationLoop_Trampoline());
            ff->f_listening.setValue(true);

            if (args.size() == 1) ff->setName(py::cast<std::string>(args[0]));

            return ff;
        }));

    f.def_property_readonly("parameters", [](sofadiff::GradientBasedOptimizationLoop& self) {
        std::vector<sofadiff::BaseParameter*> parameters;
        self.getContext()->get<sofadiff::BaseParameter>(&parameters, core::objectmodel::BaseContext::SearchDown);
        return parameters;
    });

    // f.def("initAndLinkParameter",
    //       py::overload_cast<core::objectmodel::BaseData*, core::objectmodel::BaseObject*>(
    //           &sofadiff::GradientBasedOptimizationLoop::initAndLinkParameter),
    //       py::arg("internData"), py::arg("parameter"));

    PythonFactory::registerType<sofadiff::GradientBasedOptimizationLoop>(
        [](core::objectmodel::Base* object) -> py::object
        {
            auto* sp = dynamic_cast<sofadiff::GradientBasedOptimizationLoop*>(object);
            if (!sp) return py::none();
            try
            {
                return py::cast(sp);
            }
            catch (const std::exception& e)
            {
                return py::cast(dynamic_cast<sofadiff::GradientBasedOptimizationLoop*>(object));
            }
        }
    );
}

}
