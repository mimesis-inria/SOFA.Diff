#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // Actually used

#include <SofaDiff/bindings/Binding_Parameter.h>
#include <SofaDiff/Parameter.h>

#include <SofaPython3/PythonFactory.h>
#include <SofaPython3/Sofa/Core/Binding_Base.h>


namespace py { using namespace pybind11; }

namespace sofapython3
{
using namespace sofadiff;

void declareBaseParameter(const py::module &m) {
    const auto pyclass_name = std::string("Parameter");
    py::class_<BaseParameter, py_shared_ptr<BaseParameter>> f(m, pyclass_name.c_str());
    f.def("__repr__", [](const BaseParameter & self) { return "<Parameter '" + self.getName() + "'>"; });

    f.def_property_readonly("size",
        [](const BaseParameter & self) {
            return self.getVectorSize();
        }
        );
    f.def_property_readonly("value",
        [](const BaseParameter & self) {
            const auto& value = self.getVectorFromData("value");
            return py::array(value.size(), value.data());
        }
    );
    f.def_property_readonly("gradient",
        [](const BaseParameter & self) {
            const auto& gradient = self.getVectorFromData("gradient");
            return py::array(gradient.size(), gradient.data());
        }
    );
    f.def_property("next_value",
        [](const BaseParameter & self) {
            const auto& gradient = self.getVectorFromData("nextValue");
            return py::array(gradient.size(), gradient.data());
        },
        [] (BaseParameter& self, const py::array_t<SReal>& next_value) {
            const auto v = py::cast<py::array_t<SReal>>(next_value);
            self.setDataFrom("nextValue", std::vector(v.data(), v.data() + v.size()));
        }
    );

    f.def("__getitem__", [](const BaseParameter & self, const std::string & key) {
        const auto & value = self.getVectorFromData(key);
        return py::array(value.size(), value.data());
    });
    f.def("__contains__", [](const BaseParameter & self, const std::string & key) { return self.hasData(key); });

    PythonFactory::registerType<BaseParameter>([](core::objectmodel::Base* object)
    {
        return py::cast(dynamic_cast<BaseParameter*>(object));
    });
}

void moduleAddParameter(py::module &m) {
    declareBaseParameter(m);
}

}