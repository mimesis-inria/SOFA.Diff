/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
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
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#include <sofa/diff/forcefields/ParameterizedConstantForceField.h>

#include <sofa/core/MechanicalParams.h>
#include <sofa/core/ObjectFactory.h>


namespace sofadiff
{

ParameterizedConstantForceField::ParameterizedConstantForceField()
    : d_componentFlags(initData(&d_componentFlags,
                                "componentFlags",
                                "Activation flags for [Fx,Fy,Fz,Mx,My,Mz]"))
{
}

void ParameterizedConstantForceField::init()
{

    if (auto* state = dynamic_cast<sofa::core::behavior::MechanicalState<Rigid3Types>*>(
            this->getContext()->getMechanicalState()))
    {
        this->m_systemSize = state->getSize();
    }

    Inherit::init();

    if (this->mstate.get())
        this->m_systemSize = this->mstate.get()->getSize();

    m_forceParameter = initParameter(this->d_forces);

    if (m_forceParameter)
        this->d_forces.setValue(m_forceParameter->d_value.getValue());

    if (d_componentFlags.getValue().size() != 6)
        msg_warning() << "componentFlags must have size 6.";

    checkForNotImplementedParameters(this->getDataFields());
}

void ParameterizedConstantForceField::applyParametersJacobianTranspose(
    const sofa::core::MechanicalParams*,
    sofa::core::MultiVecDerivId vecId)
{
    if (!m_forceParameter)
        return;

    const auto state = this->mstate.get();
    const auto& vectorData = *vecId[state].read();
    const sofa::helper::ReadAccessor<sofa::Data<VecDeriv>> lambda =
        sofa::helper::getReadAccessor(vectorData);

    auto grad = sofa::helper::getWriteAccessor(m_forceParameter->d_gradient);

    const auto& indices = this->d_indices.getValue();
    if (indices.size() != 1)
    {
        msg_warning() << "ParameterizedConstantForceField expects exactly one index.";
        return;
    }

    const auto idx = indices[0];
    const auto& flags = d_componentFlags.getValue();
    const auto& adj = lambda[idx];

    if (grad.empty())
        return;

    for (unsigned int i = 0; i < 6; ++i)
    {
        if (flags[i])
            grad[0][i] += adj[i];
    }
}

void ParameterizedConstantForceField::addForce(
    const core::MechanicalParams* params,
    DataVecDeriv& f,
    const DataVecCoord& x,
    const DataVecDeriv& v)
{

    if (m_forceParameter)
    {
        this->d_forces.setValue(m_forceParameter->d_value.getValue());
        this->computeForceFromForcesVector(this->d_forces.getValue());
    }

    sofa::component::mechanicalload::ConstantForceField<Rigid3Types>::addForce(params, f, x, v);
}

void registerParameterizedConstantForceField(core::ObjectFactory* factory)
{
    factory->registerObjects(
        core::ObjectRegistrationData("Constant force field differentiable wrt its componenents.")
        .add< ParameterizedConstantForceField >());
}

}
