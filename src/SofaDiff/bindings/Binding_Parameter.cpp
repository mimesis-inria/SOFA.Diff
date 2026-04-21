#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // Actually used
#include <pybind11/numpy.h>

#include <SofaDiff/bindings/Binding_Parameter.h>
#include <SofaDiff/Parameter.h>

#include <SofaPython3/PythonFactory.h>
#include <SofaPython3/Sofa/Core/Binding_Base.h>


namespace py { using namespace pybind11; }

py::array toNumpy(const std::vector<SReal> & vector)
{
    return py::array({vector.size()}, {sizeof(SReal)}, vector.data());
}

std::vector<SReal> toVector(const py::array & array)
{
    const auto v = py::cast<py::array_t<SReal>>(array);
    return std::vector(v.data(), v.data() + v.size());
}

namespace sofapython3
{
using namespace sofadiff;

void declareBaseParameter(const py::module &m) {
    const auto pyclass_name = std::string("Parameter");
    py::class_<BaseParameter, py_shared_ptr<BaseParameter>> f(m, pyclass_name.c_str());

    f.def(
        "__repr__",
        [](const BaseParameter & self) { return "<Parameter '" + self.getName() + "'>"; }
    );

    f.def_property_readonly(
        "size",
        [](const BaseParameter & self) { return self.getVectorSize(); }
    );
    f.def_property_readonly(
        "value",
        [](const BaseParameter & self) { return toNumpy(self.getVectorFromData("value")); }
    );
    f.def_property(
        "gradient",
        [](const BaseParameter & self) { return toNumpy(self.getVectorFromData("gradient")); },
        [] (BaseParameter& self, const py::array & gradient) { self.setDataFrom("gradient", toVector(gradient)); }
    );
    f.def_property(
        "next_value",
        [](const BaseParameter & self) { return toNumpy(self.getVectorFromData("nextValue")); },
        [] (BaseParameter& self, const py::array & next_value) { self.setDataFrom("nextValue", toVector(next_value)); }
    );

    f.def(
        "__getitem__",
        [](const BaseParameter & self, const std::string & key) { return toNumpy(self.getVectorFromData(key)); }
    );
    f.def(
        "__contains__",
        [](const BaseParameter & self, const std::string & key) { return self.hasData(key); }
    );

    PythonFactory::registerType<BaseParameter>([](core::objectmodel::Base* object)
    {
        return py::cast(dynamic_cast<BaseParameter*>(object));
    });
}

void moduleAddParameter(py::module &m) {
    declareBaseParameter(m);
}

}