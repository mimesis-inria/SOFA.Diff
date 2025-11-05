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

#include <ranges>  // for iterating on maps with std::views

#include <SofaDiff/GradientDescentController.h>
#include <SofaDiff/visitors/MechanicalAccumulateVecDeriv.h>
#include <SofaDiff/visitors/MechanicalPropagateVecDeriv.h>
#include <SofaDiff/ParameterizedForceField.h>
#include <SofaDiff/LossState.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/VectorOperations.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalResetForceVisitor.h>



namespace sofadiff {

using namespace simulation::mechanicalvisitor;


core::MultiVecDerivId s_geometricGradientId = core::TMultiVecId<core::VecType::V_DERIV, core::VecAccess::V_WRITE>();
core::MultiVecDerivId s_physicalGradientId = core::TMultiVecId<core::VecType::V_DERIV, core::VecAccess::V_WRITE>();


GradientDescentController::GradientDescentController()
{
    this->f_listening.setValue(true); // TODO: Why?
}


void GradientDescentController::init()
{
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


void GradientDescentController::onEndAnimationStep(const double /*dt*/)
{
    auto *ctx = this->getContext();
    auto *params = core::mechanicalparams::defaultInstance();

    // Compute the gradient of the loss wrt the parameters
    MechanicalResetForceVisitor(params, s_geometricGradientId).execute(ctx, false);
    resetParametersGradient();
    MechanicalAccumulateVecDeriv(params, s_geometricGradientId).execute(ctx, false);
    solveForPhysicalGradient();
    MechanicalPropagateVecDeriv(params, s_physicalGradientId).execute(ctx, false);
    for (const auto &forceField: m_parameterizedForceFields)
        forceField->applyParametersJacobianTranspose(params, s_physicalGradientId);

    // Update the parameters with a step of gradient descent
    updateParameters();
}


SReal GradientDescentController::getHyperparameter(const BaseParameter *parameter, const std::string &hyperparameterName) const
{
    const auto * baseData = parameter->findData(hyperparameterName);
    if (baseData == nullptr)
    {
        msg_error() << "Parameter " << parameter->getName() << " requires a data " << hyperparameterName;
        return 0.0;
    }

    const auto * data = dynamic_cast<const Data<SReal>*>(baseData);
    if (data == nullptr)
    {
        // Unlikely to trigger since the string value of the hyperparameter is converted to a double with std::stod()
        msg_error() << "Hyperparameter " << hyperparameterName << " of parameter " << parameter->getName() << " should be scalar";
        return 0.0;
    }

    return data->getValue();
}


void GradientDescentController::updateParameters()
{
    for (const auto parameter : m_trainableParameters)
    {
        auto value = parameter->getValueVector();
        const auto learningRate = getHyperparameter(parameter, "learningRate");
        const auto gradient = parameter->getGradientVector();
        for (size_t j = 0; j < gradient.size(); j++)
            value[j] -= learningRate * gradient[j];
        parameter->setValueVector(value);
    }
}


void GradientDescentController::initializeLossGradientToOne()
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


void GradientDescentController::resetParametersGradient() const
{
    for (auto & parameter : m_trainableParameters)
        parameter->resetGradient();
}


void GradientDescentController::solveForPhysicalGradient() const
{
    auto *linearSolver = l_linearSolver.get();
    linearSolver->setSystemLHVector(s_physicalGradientId);
    linearSolver->setSystemRHVector(s_geometricGradientId);
    linearSolver->solveSystem();
    // TODO: clarify why the operation below is not required
    // simulation::common::VectorOperations vop(params, ctx);
    // vop.v_eq(m_forceGradientVecId, m_forceGradientVecId, -1.0);
}


void registerGradientDescentController(core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Controller that performs gradient descent.").add< GradientDescentController >());
}


}
