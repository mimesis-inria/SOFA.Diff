#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/ParameterizedForceField.h>

#include <SofaPython3/PythonFactory.h>
#include <SofaPython3/Sofa/Core/Binding_Base.h>
#include <sofa/core/ExecParams.h>


namespace sofapython3
{
using namespace sofadiff;
namespace py { using namespace pybind11; }

template<class T>
class ParameterizedForceField_Trampoline : public ParameterizedForceField<T>
{
public:
    SOFA_CLASS(ParameterizedForceField_Trampoline, SOFA_TEMPLATE(ParameterizedForceField, T));
    using core::behavior::ForceField<T>::mstate;
    using core::behavior::ForceField<T>::getContext;
    using typename core::behavior::ForceField<T>::DataTypes;
    using typename core::behavior::ForceField<T>::Coord;
    using typename core::behavior::ForceField<T>::DataVecDeriv;
    using typename core::behavior::ForceField<T>::DataVecCoord;

    /* Inherit the constructors */
    using ParameterizedForceField<T>::ParameterizedForceField;
    static py_shared_ptr<ParameterizedForceField_Trampoline> create(const py::args& args, const py::kwargs& kwargs);

    const BaseParameter * getParameter(const std::string& name, const BaseParameter * defaultValue = nullptr);

    void init() override;
    std::string getClassName() const override;

    void addForce(const sofa::core::MechanicalParams* mparams, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& v) override;
    void addDForce(const sofa::core::MechanicalParams* mparams, DataVecDeriv& df, const DataVecDeriv& dx ) override;

    pybind11::object _addKToMatrix(const sofa::core::MechanicalParams* mparams, int nNodes, int nDofs);
    void addKToMatrix(const sofa::core::MechanicalParams* mparams, const sofa::core::behavior::MultiMatrixAccessor* dfId) override;

    SReal getPotentialEnergy(const sofa::core::MechanicalParams* /*mparams*/, const DataVecCoord& /*x*/) const override { return 0.0; }

    void applyParametersJacobianTranspose(const core::MechanicalParams* mparams, const core::MultiVecDerivId vecId) override;
};

void moduleAddParameterizedForceField(pybind11::module& m);

}
