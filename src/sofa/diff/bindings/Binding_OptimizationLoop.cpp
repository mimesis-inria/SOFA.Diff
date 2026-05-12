/******************************************************************************
*                                 SOFA.Diff                                   *
*                              (c) 2026 INRIA                                 *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: Léo Bois, Paul Baksic                                              *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <pybind11/stl.h>

#include <sofa/diff/bindings/Binding_OptimizationLoop.h>

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
    py::gil_scoped_acquire gil;
    const py::function py_override = py::get_override(this, "allocate");
    if (!py_override)
        return;  // throw std::runtime_error("allocate() not implemented in Python subclass");
    (void) py_override();  // The cast tells the IDE that discarding the return value is intentional
}

void OptimizationLoop_Trampoline::_initialize()
{
    py::gil_scoped_acquire gil;
    const py::function py_override = py::get_override(this, "initialize");
    if (!py_override)
        return;  // throw std::runtime_error("initialize() not implemented in Python subclass");
    (void) py_override();  // The cast tells the IDE that discarding the return value is intentional
}

void OptimizationLoop_Trampoline::_updateParameters()
{
    py::gil_scoped_acquire gil;
    const py::function py_override = py::get_override(this, "update_parameters");
    if (!py_override)
        throw std::runtime_error("update_parameters() not implemented in Python subclass");
    (void) py_override();  // The cast tells the IDE that discarding the return value is intentional
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
    // Documentation for the customizable methods
    f.def("allocate", [](OptimizationLoop&) {});
    f.def("initialize", [](OptimizationLoop&) {});
    f.def("update_parameters", [](OptimizationLoop&) { throw std::runtime_error("update_parameters() must be overridden in a subclass");});
    // Properties
    f.def_property("start_with_update",
        [](const OptimizationLoop& self)             { return self.d_startWithUpdate.getValue(); },
        [](OptimizationLoop& self, const bool value) { self.d_startWithUpdate.setValue(value); }
    );
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
