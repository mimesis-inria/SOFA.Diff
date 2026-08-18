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

#include <sofa/diff/forcefields/ParameterizedHeterogeneousBeamFEMForceField.h>

#include <sofa/core/MechanicalParams.h>
#include <sofa/core/ObjectFactory.h>


namespace sofadiff
{

void ParameterizedHeterogeneousBeamFEMForceField::init()
{
    HeterogeneousBeamFEMForceField::init();
    m_youngModulusParameter = initParameter(d_sectionYoung);
    checkForNotImplementedParameters(this->getDataFields());
}


void ParameterizedHeterogeneousBeamFEMForceField::applyParametersJacobianTranspose(const core::MechanicalParams* mparams, const core::MultiVecDerivId vecId)
{   
    
    if (m_youngModulusParameter == nullptr)
    {
        msg_warning() << "ParameterizedHeterogeneousBeamFEMForceField::applyParametersJacobianTranspose() skipped: no parameter to optimize. Consider using HeterogeneousBeamFEMForceField instead. Or not?";
        return;
    }

    const auto state = this->mstate.get();

    // Get vector to multiply with
    const auto & vectorData = *vecId[state].read();
    const helper::ReadAccessor<Data<VecDeriv>> vector = helper::getReadAccessor(vectorData);
    const auto& sectionYoung = this->d_sectionYoung.getValue();
    auto youngModulusGradient = helper::getWriteAccessor(m_youngModulusParameter->d_gradient);
    youngModulusGradient.resize(sectionYoung.size());

    const VecCoord & x = state->readPositions().ref();   
    
    typename VecElement::const_iterator it;
    unsigned int i;
    
    for(it=m_indexedElements->begin(),i=0; it!=m_indexedElements->end(); ++it,++i)
    {
        const int s = this->getSectionForEdge(i);
        if (s < 0) continue;

        const auto& [a, b] = it->array();

        this->initLarge(i, a, b); // keep same pre-step as addForce

        Deriv fa, fb;
        this->LocalForceLarge(x, i, a, b, fa, fb);

        youngModulusGradient[size_t(s)] += (sofa::type::dot(fa, vector[a]) + sofa::type::dot(fb, vector[b])) / sectionYoung[size_t(s)];
    }
}

void registerParameterizedHeterogeneousBeamFEMForceField(core::ObjectFactory* factory)
{
    factory->registerObjects(
        core::ObjectRegistrationData("Heterogeneous Beam FEM force field differentiable wrt Young modulus.")
        .add< ParameterizedHeterogeneousBeamFEMForceField >());
}

}
