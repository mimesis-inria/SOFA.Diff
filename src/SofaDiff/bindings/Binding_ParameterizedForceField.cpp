#include <pybind11/stl.h>

#include <SofaDiff/bindings/Binding_ParameterizedForceField.h>
#include <SofaDiff/bindings/Binding_OptimizationLoop.h>

#include <sofa/core/MechanicalParams.h>
#include <sofa/core/behavior/DefaultMultiMatrixAccessor.h>
#include <SofaPython3/Sofa/Core/Binding_Base.h>
#include <SofaPython3/DataHelper.h>
#include <utility>


namespace sofapython3
{
namespace py { using namespace pybind11; }
using namespace py::literals;

template<class T>
py_shared_ptr<ParameterizedForceField_Trampoline<T>> ParameterizedForceField_Trampoline<T>::create(const py::args& args, const py::kwargs& kwargs)
{
    auto object = py_shared_ptr(new ParameterizedForceField_Trampoline<T>());
    object->f_listening.setValue(true);

    if (!args.empty())
    {
        msg_warning("ParameterizedForceField") << "Positional arguments were passed to the constructor, but only keyword arguments are handled";
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

template<class T>
const BaseParameter * ParameterizedForceField_Trampoline<T>::getParameter(const std::string& name, const BaseParameter * defaultValue)
{
    py::gil_scoped_acquire gil_acquire;

    auto * baseData = this->findData(name);
    if (baseData == nullptr)
    {
        msg_warning() << "Data '" << name << "' not found.";
        return nullptr;
    }

    auto * data = dynamic_cast<Data<SReal>*>(baseData);
    if (data == nullptr)
    {
        msg_warning() << "Data '" << name << "' does not have the correct type";
        return nullptr;
    }

    auto * parameter = this->initParameter(*data);
    if (parameter == nullptr)
        return defaultValue;

    return static_cast<BaseParameter*>(parameter);
}

template<class T>
void ParameterizedForceField_Trampoline<T>::init()
{
    ParameterizedForceField<T>::init();

    py::gil_scoped_acquire gil_acquire;

    if (!mstate.get())
        mstate.set(dynamic_cast<behavior::MechanicalState<DataTypes>*>(getContext()->getMechanicalState()));

    if(!mstate.get())
        throw py::type_error("Missing mechanical state.");

    PYBIND11_OVERLOAD(void, ParameterizedForceField<T>, init, );
}

/* -------------------------------------------------------------------------------------------------------------- *
 * Copy-paste from Binding_ForceField.cpp (pybind11 says we have to do that — I think)                            *
 * (https://pybind11.readthedocs.io/en/stable/advanced/classes.html#combining-virtual-functions-and-inheritance)  *
 * -------------------------------------------------------------------------------------------------------------- */

template<class T>
void ParameterizedForceField_Trampoline<T>::addForce(const MechanicalParams* mparams,  DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& v)
{
    py::gil_scoped_acquire gil_acquire;
    py::dict mp = py::dict("time"_a=getContext()->getTime(),
                           "mFactor"_a=mparams->mFactor(),
                           "bFactor"_a=mparams->bFactor(),
                           "kFactor"_a=mparams->kFactor(),
                           "isImplicit"_a=mparams->implicit(),
                           "energy"_a=mparams->energy());
    PYBIND11_OVERLOAD_PURE(void, ParameterizedForceField<T>, addForce, mp, PythonFactory::toPython(&f), PythonFactory::toPython(&x), PythonFactory::toPython(&v));
}

template<class T>
void ParameterizedForceField_Trampoline<T>::addDForce(const MechanicalParams* mparams, DataVecDeriv& df, const DataVecDeriv& dx )
{
    py::gil_scoped_acquire gil_acquire;
    py::dict mp = py::dict("time"_a=getContext()->getTime(),
                           "nFactor"_a=mparams->mFactor(),
                           "bFactor"_a=mparams->bFactor(),
                           "kFactor"_a=mparams->kFactor(),
                           "isImplicit"_a=mparams->implicit()
                           );
    PYBIND11_OVERLOAD_PURE(void, ParameterizedForceField<T>, addDForce, mp, PythonFactory::toPython(&df), PythonFactory::toPython(&dx));
}

template<class T>
py::object ParameterizedForceField_Trampoline<T>::_addKToMatrix(const MechanicalParams* mparams, int nIndices, int nDofs)
{
    py::dict mp = py::dict("time"_a=getContext()->getTime(),
                           "mFactor"_a=mparams->mFactor(),
                           "bFactor"_a=mparams->bFactor(),
                           "kFactor"_a=mparams->kFactor(),
                           "isImplicit"_a=mparams->implicit()
            );
    PYBIND11_OVERLOAD_PURE(py::object, ParameterizedForceField<T>, addKToMatrix, mp, nIndices, nDofs);
}

template<class T>
void ParameterizedForceField_Trampoline<T>::addKToMatrix(const MechanicalParams* mparams, const core::behavior::MultiMatrixAccessor* dfId)
{
    py::gil_scoped_acquire gil_acquire;

    core::behavior::MultiMatrixAccessor::MatrixRef mref = dfId->getMatrix(this->mstate);
    sofa::linearalgebra::BaseMatrix* mat = mref.matrix;

    size_t offset = mref.offset;
    // nNodes is the number of nodes (positions) of the object whose K matrix we're computing
    int nNodes = int(mparams->readX(mstate.get())->getValue().size());
    // nDofs is the number of degrees of freedom per-element of the object whose K matrix we're computing
    int nDofs = Coord::total_size;

    py::object ret = _addKToMatrix(mparams, nNodes, nDofs);

    if(!py::isinstance<py::array>(ret))
    {
        throw py::type_error("Can't read return value of AddKToMatrix. A numpy array is expected");
    }

    // if ret is numpy array
    auto r = py::cast<py::array>(ret);
    if (r.ndim() == 3 && r.shape(2) == 1)
    {
        // read K as a plain 2D matrix
        auto kMatrix = r.unchecked<double, 3>();
        for (size_t x = 0 ; x < size_t(kMatrix.shape(0)) ; ++x)
        {
            for (size_t y = 0 ; y < size_t(kMatrix.shape(1)) ; ++y)
            {
                mat->add(int(offset + x), int(offset + y), kMatrix(x,y, 0));
            }
        }
    }
    else if (r.ndim() == 2 && r.shape(1) == 3)
    {
        // consider ret to be a list of tuples [(i,j,[val])]
        auto kMatrix = r.unchecked<double, 2>();
        for (auto x = 0 ; x < kMatrix.shape(0) ; ++x)
        {
            mat->add(int(offset + size_t(kMatrix(x,0))), int(offset + size_t(kMatrix(x,1))), kMatrix(x,2));
        }
    }
    else
    {
        throw py::type_error("Can't read return value of AddKToMatrix. The method should return either a plain 2D matrix or a vector of tuples (i, j, val)");
    }
}

/* -------------- *
 * The new method *
 * -------------- */

template <class T>
void ParameterizedForceField_Trampoline<T>::applyParametersJacobianTranspose(const MechanicalParams* mparams, const MultiVecDerivId vecId)
{
    py::gil_scoped_acquire gil;
    const py::function py_override = py::get_override(this, "propagate_gradient_to_parameters");
    if (!py_override)
        throw std::runtime_error("propagate_gradient_to_parameters(self, force_gradient) not implemented in Python subclass");
    const auto state = this->mstate.get();
    const auto & data = *vecId[state].read();
    (void) py_override(PythonFactory::toPython(&data));  // The cast tells the IDE that discarding the return value is intentional
}

template<class T>
std::string ParameterizedForceField_Trampoline<T>::getClassName() const
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
        return "trampoline_ParameterizedForceField_cast_err";
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error in getClassName: " << e.what() << std::endl;
        return "trampoline_ParameterizedForceField_err";
    }
}

template<class T>
void declare_forcefield(py::module &m) {
    const std::string pyclass_name = std::string("ParameterizedForceField") + T::Name();

    py::class_<ParameterizedForceField<T>, objectmodel::BaseObject, ParameterizedForceField_Trampoline<T>, py_shared_ptr<ParameterizedForceField<T>>>
    f(m, pyclass_name.c_str(), py::dynamic_attr(), py::multiple_inheritance());

    f.def(py::init(&ParameterizedForceField_Trampoline<T>::create));
    f.def("get_parameter", [](ParameterizedForceField_Trampoline<T>& self, const std::string & name) -> const BaseParameter* {
        return self.getParameter(name, nullptr);
    });
    f.def("get_parameter", [](ParameterizedForceField_Trampoline<T>& self, const std::string & name, const BaseParameter * defaultValue) -> const BaseParameter* {
        return self.getParameter(name, defaultValue);
    });
    f.def("init", [](ParameterizedForceField_Trampoline<T>& self) { self.init(); });
    // TODO: add interface documentation?

    PythonFactory::registerType<ParameterizedForceField<T>>([](sofa::core::objectmodel::Base* object)
    {
        return py::cast(dynamic_cast<ParameterizedForceField<T>*>(object));
    });
}

void moduleAddParameterizedForceField(py::module &m) {
    py::class_<Parameterized, objectmodel::BaseObject, py_shared_ptr<Parameterized>>
        (m, "Parameterized", py::dynamic_attr(), py::multiple_inheritance());
    declare_forcefield<defaulttype::Vec3dTypes>(m);
    declare_forcefield<defaulttype::Vec2dTypes>(m);
    declare_forcefield<defaulttype::Vec1dTypes>(m);
    declare_forcefield<defaulttype::Vec6dTypes>(m);
    declare_forcefield<defaulttype::Rigid3dTypes>(m);
    declare_forcefield<defaulttype::Rigid2dTypes>(m);
}

}
