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
#include <SofaDiff/visitors/AdjointResetVisitor.h>
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

void registerDifferentiableAnimationLoop(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Controller that performs gradient descent.").add< DifferentiableAnimationLoop >());
}

DifferentiableAnimationLoop::DifferentiableAnimationLoop():
    d_differentiableMode(initData(&d_differentiableMode, "differentiableMode", "whether differentiable mode is ON or OFF"))
{}

void DifferentiableAnimationLoop::init()
{
    DefaultAnimationLoop::init();
    LinearSolverAccessor::init();

    auto* ctx = this->getContext();
    auto* params = mechanicalparams::defaultInstance();
    simulation::common::VectorOperations vop(params, ctx);

    m_timestepIndex = 0;
    m_timestepTotal = 0;
    d_differentiableMode.setValue(false);

    ctx->get<LossState> (&m_lossStates, BaseContext::SearchRoot);
}


void DifferentiableAnimationLoop::resetDifferentiableMode()
{
    m_timestepIndex = 0;
    m_timestepTotal = 0;
}


void DifferentiableAnimationLoop::storeState(const ExecParams* params)
{
    m_timestepIndex++;
    if (m_node == nullptr)
    {
        m_node = dynamic_cast<simulation::Node*>(this->l_node.get());
        if (m_node == nullptr)
        {
            msg_error("Impossible to get root node");
        }
    }
    if (m_timestepIndex > m_positionStorage.size() + 1)
    {
        msg_error("Storage requires more than one additional VecId — This should not happen.");
        return;
    }
    if (m_timestepIndex == m_positionStorage.size() + 1)
    {
        auto* ctx = this->getContext();
        simulation::common::VectorOperations vop(params, ctx);

        behavior::MultiVecCoord position(&vop, TMultiVecId<VecType::V_COORD, VecAccess::V_WRITE>());
        position.realloc(&vop, false, true, VecIdProperties{"x"+std::to_string(m_timestepIndex - 1), this->getClassName()});

        behavior::MultiVecDeriv velocity(&vop, TMultiVecId<VecType::V_DERIV, VecAccess::V_WRITE>());
        velocity.realloc(&vop, false, true, VecIdProperties{"v"+std::to_string(m_timestepIndex - 1), this->getClassName()});

        m_positionStorage.push_back(position.id());
        m_velocityStorage.push_back(velocity.id());
    }
    StoreStateVisitor(params, m_positionStorage[m_timestepIndex - 1], m_velocityStorage[m_timestepIndex - 1]).execute(m_node);
}

void DifferentiableAnimationLoop::retrieveState(const ExecParams* params)
{
    auto m_params = MechanicalParams(*params);
    RetrieveStateVisitor(params, m_positionStorage[m_timestepIndex - 1], m_velocityStorage[m_timestepIndex - 1]).execute(m_node);
    MechanicalPropagateOnlyPositionAndVelocityVisitor(&m_params).execute(m_node);
    // simulation::UpdateMappingVisitor(params).execute(m_node); // TODO: useful?
    m_timestepIndex--;
}


void DifferentiableAnimationLoop::step(const ExecParams* params, SReal dt)
{
    if (!d_differentiableMode.getValue())
    {
        DefaultAnimationLoop::step(params, dt);
    }
    else
    {
        if (m_solverDirection != FORWARD)
        {
            m_timestepTotal = m_timestepIndex;
            m_solverDirection = FORWARD;
        }
        storeState(params);
        DefaultAnimationLoop::step(params, dt);
        m_timestepTotal++;
    }
}


void DifferentiableAnimationLoop::stepAdjoint(const ExecParams* params, SReal dt)
{
    if (this->d_componentState.getValue() != objectmodel::ComponentState::Valid)
    {
        return;
    }

    if (m_timestepIndex < 1)
    {
        return;
    }

    if (m_solverDirection != BACKWARD)
    {
        m_solverDirection = BACKWARD;
        AdjointResetVisitor(params).execute(m_node);
    }

    // Hard coded "global loss" equal to the latest "instant loss": very dirty and temporary
    setLossGradient(1.0);

    // TODO: use the other constructor (like DefaultAnimationLoop with SolveVisitor)
    AdjointSolveVisitor(params, dt, vec_id::write_access::position, vec_id::write_access::velocity).execute(m_node);
    updateSimulationContext(params, -dt, m_node->getTime());
    retrieveState(params);
}


void DifferentiableAnimationLoop::setLossGradient(const SReal value)
{
    for (const auto loss : m_lossStates)
    {
        if (loss == nullptr)
        {
            msg_error() << "Bad link to the loss object";
            this->d_componentState.setValue(ComponentState::Invalid);
            return;
        }
        // TODO: handle case where we cannot write in gradient
        helper::WriteAccessor<Data<VecDeriv_t<defaulttype::Vec1Types>> > lossGradient = helper::getWriteAccessor(loss->d_gradient);
        lossGradient[0] = sofa::Deriv_t<defaulttype::Vec1Types> (value);
    }
}

void DifferentiableAnimationLoop::setDifferentiableMode(const bool differentiable)
{
    if (d_differentiableMode.getValue() && !differentiable) // Deactivate differentiable mode
    {
        resetDifferentiableMode();
    }
    d_differentiableMode.setValue(differentiable);
}

}
