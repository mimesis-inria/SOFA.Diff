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
#include <SofaDiff/visitors/MechanicalAccumulateVecDeriv.h>
#include <SofaDiff/visitors/MechanicalPropagateVecDeriv.h>
#include <SofaDiff/ParameterizedForceField.h>
#include <SofaDiff/LossState.h>

#include <sofa/simulation/VectorOperations.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalResetForceVisitor.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/core/ObjectFactory.h>


namespace sofadiff
{

using namespace simulation::mechanicalvisitor;


core::MultiVecDerivId s_geometricGradientId = core::TMultiVecId<core::VecType::V_DERIV, core::VecAccess::V_WRITE>();
core::MultiVecDerivId s_physicalGradientId = core::TMultiVecId<core::VecType::V_DERIV, core::VecAccess::V_WRITE>();


void DifferentiableAnimationLoop::init()
{
    DefaultAnimationLoop::init();
    LinearSolverAccessor::init();

    auto* ctx = this->getContext();
    auto* params = core::mechanicalparams::defaultInstance();
    simulation::common::VectorOperations vop(params, ctx);

    core::behavior::MultiVecDeriv geometricGradient(&vop, s_geometricGradientId);
    geometricGradient.realloc(&vop, false, true, core::VecIdProperties{"Geometric gradient of the loss", this->getClassName()});
    s_geometricGradientId = geometricGradient.id();

    core::behavior::MultiVecDeriv physicalGradient(&vop, s_physicalGradientId);
    physicalGradient.realloc(&vop, false, true, core::VecIdProperties{"Physical gradient of the loss", this->getClassName()});
    s_physicalGradientId = physicalGradient.id();

    ctx->get<BaseParameter> (&m_trainableParameters, BaseContext::SearchRoot);
    ctx->get<Parameterized> (&m_parameterizedForceFields, BaseContext::SearchRoot);
    ctx->get<LossState> (&m_lossStates, BaseContext::SearchRoot);

    initializeLossGradientToOne();
}


void DifferentiableAnimationLoop::step(const sofa::core::ExecParams* params, SReal dt)
{
    DefaultAnimationLoop::step(params, dt);
    stepAdjoint(params, dt); // temporary: later on should be called directly by user (e.g. button in GUI)
}

void DifferentiableAnimationLoop::stepAdjoint(const core::ExecParams* params, SReal dt)
{
    if (this->d_componentState.getValue() != ComponentState::Valid)
    {
        return;
    }

    auto *ctx = this->getContext();
    const core::MechanicalParams mparams(*params);

    // Compute the gradient of the loss wrt the parameters
    MechanicalResetForceVisitor(&mparams, s_geometricGradientId).execute(ctx, false);
    resetParametersGradient();
    MechanicalAccumulateVecDeriv(&mparams, s_geometricGradientId).execute(ctx, false);
    solveForPhysicalGradient();
    MechanicalPropagateVecDeriv(&mparams, s_physicalGradientId).execute(ctx, false);
    {
        SCOPED_TIMER("GradientDescentController::applyParametersJacobianTranspose");
        for (const auto &forceField: m_parameterizedForceFields)
            forceField->applyParametersJacobianTranspose(&mparams, s_physicalGradientId);
    }
}

void DifferentiableAnimationLoop::initializeLossGradientToOne()
{
    for (const auto loss : m_lossStates)
    {
        if (loss == nullptr)
        {
            msg_error() << "Bad link to the loss object";
            this->d_componentState.setValue(ComponentState::Invalid);
            return;
        }
        const auto& gradient = s_geometricGradientId.getId(loss);
        // TODO: handle case where we cannot write in gradient
        helper::WriteAccessor<Data<VecDeriv_t<defaulttype::Vec1Types>> > lossGradient = loss->write(gradient);
        lossGradient[0] = sofa::Deriv_t<defaulttype::Vec1Types> (1);
    }
}


void DifferentiableAnimationLoop::resetParametersGradient() const
{
    SCOPED_TIMER("GradientDescentController::resetParametersGradient");

    for (auto & parameter : m_trainableParameters)
        parameter->resetGradient();
}


void DifferentiableAnimationLoop::solveForPhysicalGradient()
{
    SCOPED_TIMER("GradientDescentController::solveForPhysicalGradient");
    auto *linearSolver = l_linearSolver.get();

    const auto * params = core::execparams::defaultInstance();
    simulation::common::MechanicalOperations mop(params, this->getContext());
    mop->setImplicit(true);
    static constexpr core::MatricesFactors::M m(0);
    static constexpr core::MatricesFactors::B b(0);
    static constexpr core::MatricesFactors::K k(-1);
    mop.setSystemMBKMatrix(m, b, k, linearSolver);

    linearSolver->getLinearSystem()->setSystemSolution(s_physicalGradientId);
    linearSolver->getLinearSystem()->setRHS(s_geometricGradientId);
    linearSolver->solveSystem();  // Solve -df/dx * lambda = dy/dx
    linearSolver->getLinearSystem()->dispatchSystemSolution(s_physicalGradientId);
}


void registerDifferentiableAnimationLoop(core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Controller that performs gradient descent.").add< DifferentiableAnimationLoop >());
}

}