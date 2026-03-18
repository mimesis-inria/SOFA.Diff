#include <pybind11/pybind11.h>

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

    f.def_property("next_value",
        [](const BaseParameter & self) {
            const auto& next_value = self.getNextValueVector();
            return py::array(next_value.size(), next_value.data());
        },
        [] (BaseParameter& self, const py::array_t<SReal>& next_value) {
            const auto v = py::cast<py::array_t<SReal>>(next_value);
            self.setNextValueVector(std::vector(v.data(), v.data() + v.size()));
        }
    );
    f.def_property_readonly("value",
        [](const BaseParameter & self) {
            const auto& value = self.getValueVector();
            return py::array(value.size(), value.data());
        }
    );
    f.def_property_readonly("gradient",
        [](const BaseParameter & self) {
            const auto& gradient = self.getGradientVector();
            return py::array(gradient.size(), gradient.data());
        }
    );

    f.def("__getitem__", [](const BaseParameter & self, const std::string & key) { return self.getHyperparameter(key); });
    f.def("__contains__", [](const BaseParameter & self, const std::string & key) { return self.hasHyperparameter(key); });

    PythonFactory::registerType<BaseParameter>([](core::objectmodel::Base* object)
    {
        return py::cast(dynamic_cast<BaseParameter*>(object));
    });
}

void moduleAddParameter(py::module &m) {
    declareBaseParameter(m);
}

}