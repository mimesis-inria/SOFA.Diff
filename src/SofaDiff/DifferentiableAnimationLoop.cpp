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

#include <SofaDiff/DifferentiableAnimationLoop.h>
#include <SofaDiff/visitors/AdjointSolveVisitor.h>
#include <SofaDiff/ParameterizedForceField.h>

#include <sofa/core/MechanicalParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalResetForceVisitor.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/VectorOperations.h>


namespace sofadiff
{
using namespace sofa::simulation::mechanicalvisitor;
using namespace sofa::core;


MultiVecDerivId s_geometricGradientId = TMultiVecId<VecType::V_DERIV, VecAccess::V_WRITE>();
MultiVecDerivId s_physicalGradientId = TMultiVecId<VecType::V_DERIV, VecAccess::V_WRITE>();


void DifferentiableAnimationLoop::init()
{
    DefaultAnimationLoop::init();
    LinearSolverAccessor::init();

    auto* ctx = this->getContext();
    auto* params = mechanicalparams::defaultInstance();
    simulation::common::VectorOperations vop(params, ctx);

    behavior::MultiVecDeriv geometricGradient(&vop, s_geometricGradientId);
    geometricGradient.realloc(&vop, false, true, VecIdProperties{"Geometric gradient of the loss", this->getClassName()});
    s_geometricGradientId = geometricGradient.id();

    behavior::MultiVecDeriv physicalGradient(&vop, s_physicalGradientId);
    physicalGradient.realloc(&vop, false, true, VecIdProperties{"Physical gradient of the loss", this->getClassName()});
    s_physicalGradientId = physicalGradient.id();
}


void DifferentiableAnimationLoop::step(const ExecParams* params, SReal dt)
{
    DefaultAnimationLoop::step(params, dt);
    stepAdjoint(params, dt); // temporary: later on should be called directly by user (e.g. button in GUI)
}


void DifferentiableAnimationLoop::stepAdjoint(const ExecParams* params, SReal dt)
{
    if (this->d_componentState.getValue() != objectmodel::ComponentState::Valid)
    {
        return;
    }

    // TODO: call the "backward solver" if dynamic

    // TODO: use the other constructor (like DefaultAnimationLoop with SolveVisitor)
    AdjointSolveVisitor(params, dt, vec_id::write_access::position, vec_id::write_access::velocity).execute(m_node);
}


void registerDifferentiableAnimationLoop(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Controller that performs gradient descent.").add< DifferentiableAnimationLoop >());
}

}
