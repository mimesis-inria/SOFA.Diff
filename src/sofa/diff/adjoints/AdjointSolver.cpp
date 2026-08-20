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
#include <sofa/diff/adjoints/AdjointSolver.h>

#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/VectorOperations.h>

#include <sofa/component/solidmechanics/fem/elastic/BeamFEMForceField.h>

namespace sofadiff
{

using namespace sofa::core;
using namespace sofa::core::objectmodel;

void AdjointSolver::init()
{
    const auto* ctx = this->getContext();
    ctx->get<BaseParameter> (&m_trainableParameters, BaseContext::SearchRoot);
    ctx->get<Parameterized> (&m_parameterizedForceFields, BaseContext::SearchRoot);
    ctx->get<LossState> (&m_lossStates, BaseContext::SearchRoot);
}

void AdjointSolver::bwdInit()
{
    // Tell the lossStates that they should return their d_gradient when asked for the given VecId
    for (const auto loss : m_lossStates)
        loss->setGradientVecId(getLossGradientId());
}


MultiVecDerivId AdjointSolver::newVecId(const char * name)
{
    simulation::common::VectorOperations vop(mechanicalparams::defaultInstance(), this->getContext()->getRootContext());
    const auto vecId = TMultiVecId<VecType::V_DERIV, VecAccess::V_WRITE>();
    behavior::MultiVecDeriv vec(&vop, vecId);
    vec.realloc(&vop, false, true, VecIdProperties{name, this->getClassName()});
    return vec.id();
}

void AdjointSolver::resetParametersGradient() const
{
    SCOPED_TIMER("AdjointSolver::resetParametersGradient");
    for (auto & parameter : m_trainableParameters)
        parameter->setDataFrom("gradient", 0);
}

void AdjointSolver::propagateGradientsThroughForceFields(const MechanicalParams* mparams, const MultiVecDerivId& forceGradientId) const
{
    SCOPED_TIMER("AdjointSolver::propagateGradientsThroughForceFields");

    using BeamFF = sofa::component::solidmechanics::fem::elastic::BeamFEMForceField<sofa::defaulttype::Rigid3Types>;
    for (const auto& forceField : m_parameterizedForceFields)
    {
        forceField->applyParametersJacobianTranspose(mparams, forceGradientId);
        if (auto* beam = dynamic_cast<BeamFF*>(forceField))
            beam->reinit();
    }
}

}
