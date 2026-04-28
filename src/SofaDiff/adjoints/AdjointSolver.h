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
#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/LossState.h>
#include <SofaDiff/Parameter.h>
#include <SofaDiff/ParameterizedForceField.h>

#include <sofa/core/MechanicalParams.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/core/objectmodel/BaseObject.h>


namespace sofadiff
{
using namespace sofa::core;


class SOFA_SOFADIFF_API AdjointSolver: public virtual objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS(AdjointSolver, BaseObject);

    // Implementation of the method toAdjointSolver(), assuming such a method was declared in Base.
    // Which is not the case, so it does not work. But what is the point of this method anyway?
    // SOFA_BASE_CAST_IMPLEMENTATION(AdjointSolver)

    void init() override;
    void bwdInit() override;

    // Called at the first stepAdjoint() of a DifferentiableAnimationLoop (beginning of the backpropagation)
    virtual void resetGradients(const ExecParams* /*params*/) = 0;

    // Called at each stepAdjoint() of a DifferentiableAnimationLoop
    virtual void solve(const ExecParams* /*params*/, SReal /*dt*/, MultiVecCoordId /*xResult*/, MultiVecDerivId /*vResult*/) = 0;

protected:
    std::vector<LossState *> m_lossStates;
    std::vector<BaseParameter *> m_trainableParameters;
    std::vector<Parameterized *> m_parameterizedForceFields;

    MultiVecDerivId newVecId(const char * name);
    void resetParametersGradient() const;
    void propagateGradientsThroughForceFields(const MechanicalParams * mparams, const MultiVecDerivId & forceGradientId) const;

    // Return the VecId that is used in MechanicalAccumulateVecDeriv to backpropagate the gradient of the loss
    virtual MultiVecDerivId & getLossGradientId() = 0;
};
}
