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
#include <SofaDiff/visitors/StoreStateVisitor.h>
#include <SofaDiff/visitors/RetrieveStateVisitor.h>
#include <SofaDiff/ParameterizedForceField.h>

#include <sofa/core/MechanicalParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/Node.h>
// #include <sofa/simulation/UpdateMappingVisitor.h>  // cf TODO below
#include <sofa/simulation/mechanicalvisitor/MechanicalPropagateOnlyPositionAndVelocityVisitor.h>
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

    m_index = 0;
}


void DifferentiableAnimationLoop::storeState(const ExecParams* params)
{
    m_index++;
    if (m_index > m_positionStorage.size() + 1)
    {
        msg_error("Storage requires more than one additional VecId — This should not happen.");
        return;
    }
    if (m_index == m_positionStorage.size() + 1)
    {
        auto* ctx = this->getContext();
        simulation::common::VectorOperations vop(params, ctx);

        behavior::MultiVecCoord position(&vop, TMultiVecId<VecType::V_COORD, VecAccess::V_WRITE>());
        position.realloc(&vop, false, true, VecIdProperties{"x"+std::to_string(m_index - 1), this->getClassName()});

        behavior::MultiVecDeriv velocity(&vop, TMultiVecId<VecType::V_DERIV, VecAccess::V_WRITE>());
        velocity.realloc(&vop, false, true, VecIdProperties{"v"+std::to_string(m_index - 1), this->getClassName()});

        m_positionStorage.push_back(position.id());
        m_velocityStorage.push_back(velocity.id());
    }
    StoreStateVisitor(params, m_positionStorage[m_index - 1], m_velocityStorage[m_index - 1]).execute(m_node);
}

void DifferentiableAnimationLoop::retrieveState(const ExecParams* params)
{
    auto m_params = MechanicalParams(*params);
    RetrieveStateVisitor(params, m_positionStorage[m_index - 1], m_velocityStorage[m_index - 1]).execute(m_node);
    MechanicalPropagateOnlyPositionAndVelocityVisitor(&m_params).execute(m_node);
    // simulation::UpdateMappingVisitor(params).execute(m_node); // TODO: useful?
    m_index--;
}


void DifferentiableAnimationLoop::step(const ExecParams* params, SReal dt)
{
    DefaultAnimationLoop::step(params, dt);
    storeState(params);
}


void DifferentiableAnimationLoop::stepAdjoint(const ExecParams* params, SReal dt)
{
    if (this->d_componentState.getValue() != objectmodel::ComponentState::Valid)
    {
        return;
    }

    if (m_index < 1)
    {
        return;
    }

    // TODO: use the other constructor (like DefaultAnimationLoop with SolveVisitor)
    AdjointSolveVisitor(params, dt, vec_id::write_access::position, vec_id::write_access::velocity).execute(m_node);
    updateSimulationContext(params, -dt, m_node->getTime());
    retrieveState(params);
}


void registerDifferentiableAnimationLoop(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Controller that performs gradient descent.").add< DifferentiableAnimationLoop >());
}

}
