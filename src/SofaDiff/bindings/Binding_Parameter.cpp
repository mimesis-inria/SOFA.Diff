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

    // f.def("get_data_in_group", &BaseParameter::getDataInGroup);
    // f.def("set_data_in_group", &BaseParameter::getDataInGroup);

    f.def_property_readonly("value",
        [](const BaseParameter & self) {
            const auto& value = self.getValue();
            return py::array(value.size(), value.data());
        }
    );
    f.def_property_readonly("gradient",
        [](const BaseParameter & self) {
            const auto& gradient = self.getGradient();
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