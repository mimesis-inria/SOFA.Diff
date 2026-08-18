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

#include <sofa/diff/forcefields/ParameterizedBeamFEMForceField.h>

#include <sofa/core/MechanicalParams.h>
#include <sofa/core/ObjectFactory.h>


namespace sofadiff
{

void ParameterizedBeamFEMForceField::init()
{
    BeamFEMForceField::init();
    m_youngModulusParameter = initParameter(d_youngModulus);
    checkForNotImplementedParameters(this->getDataFields());
}


void ParameterizedBeamFEMForceField::applyParametersJacobianTranspose(const core::MechanicalParams* mparams, const core::MultiVecDerivId vecId)
{   
    
    if (m_youngModulusParameter == nullptr)
    {
        msg_warning() << "ParameterizedBeamFEMForceField::applyParametersJacobianTranspose() skipped: no parameter to optimize. Consider using BeamFEMForceField instead. Or not?";
        return;
    }

    const auto state = this->mstate.get();

    // Get vector to multiply with
    const auto & vectorData = *vecId[state].read();
    const helper::ReadAccessor<Data<VecDeriv>> vector = helper::getReadAccessor(vectorData);

    // Get the force, since the jacobian happens to be the force divided by the Young modulus
    const VecCoord & x = state->readPositions().ref();
    const DataVecDeriv v;
    DataVecDeriv f;
    auto writeAccess = f.beginWriteOnly();
    writeAccess->resize(x.size());
    this->addForce(nullptr, f, x, v);

    // Apply the jacobian: gradient += dot(jacobian, vector)
    const auto youngModulus = this->d_youngModulus.getValue()[0];
    auto youngModulusGradient = helper::getWriteAccessor(m_youngModulusParameter->d_gradient);
    for (unsigned int i=0; i < writeAccess->size(); i++)
    {
        youngModulusGradient[0] += sofa::type::dot((*writeAccess)[i], vector[i]) / youngModulus;
    }
}

void registerParameterizedBeamFEMForceField(core::ObjectFactory* factory)
{
    factory->registerObjects(
        core::ObjectRegistrationData("Beam FEM force field differentiable wrt Young modulus.")
        .add< ParameterizedBeamFEMForceField >());
}

}
