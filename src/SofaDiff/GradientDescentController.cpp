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

#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/VectorOperations.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalResetForceVisitor.h>


namespace sofadiff {

using namespace simulation::mechanicalvisitor;


GradientDescentController::GradientDescentController()
: l_loss(initLink("loss", "Mechanical object (vec1) with the loss to minimize"))
{
    this->f_listening.setValue(true);
}


void GradientDescentController::init()
{
    LinearSolverAccessor::init();

    auto* ctx = this->getContext();
    auto* params = core::mechanicalparams::defaultInstance();
    simulation::common::VectorOperations vop(params, ctx);

    core::behavior::MultiVecDeriv geometricGradient(&vop, m_geometricGradientId);
    geometricGradient.realloc(&vop, false, true, core::VecIdProperties{"Geometric gradient of the loss", this->getClassName()});
    m_geometricGradientId = geometricGradient.id();

    core::behavior::MultiVecDeriv physicalGradient(&vop, m_physicalGradientId);
    physicalGradient.realloc(&vop, false, true, core::VecIdProperties{"Physical gradient of the loss", this->getClassName()});
    m_physicalGradientId = physicalGradient.id();

    ctx->get<TrainableParameter> (&m_trainableParameters, core::objectmodel::BaseContext::SearchRoot);
    ctx->get<Parameterized> (&m_parameterizedForceFields, core::objectmodel::BaseContext::SearchRoot);
}


void GradientDescentController::onEndAnimationStep(const double /*dt*/)
{
    auto *ctx = this->getContext();
    auto *params = core::mechanicalparams::defaultInstance();

    // Compute the gradient of the loss wrt the parameters
    MechanicalResetForceVisitor(params, m_geometricGradientId).execute(ctx, false);
    initializeLossGradientToOne();
    resetParametersGradient();
    MechanicalAccumulateVecDeriv(params, m_geometricGradientId).execute(ctx, false);
    solveForPhysicalGradient();
    MechanicalPropagateVecDeriv(params, m_physicalGradientId).execute(ctx, false);
    for (const auto &forceField: m_parameterizedForceFields)
        forceField->applyParametersJacobianTranspose(params, m_physicalGradientId);

    // Update the parameters with a step of gradient descent
    updateParameters();
}


SReal GradientDescentController::getHyperparameter(const TrainableParameter *parameter, const std::string &hyperparameterName) const
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
    const auto loss = l_loss.get();
    if (loss == nullptr)
    {
        msg_error() << "Bad link to the loss object";
        this->d_componentState.setValue(core::objectmodel::ComponentState::Invalid);
        return;
    }
    const auto& gradient = m_geometricGradientId.getId(loss);
    helper::WriteAccessor<Data<VecDeriv_t<defaulttype::Vec1Types>> > lossGradient = loss->write(gradient);
    lossGradient[0] = sofa::Deriv_t<defaulttype::Vec1Types> (1);
}


void GradientDescentController::resetParametersGradient() const
{
    for (auto & parameter : m_trainableParameters)
        parameter->resetGradient();
}


void GradientDescentController::solveForPhysicalGradient() const
{
    auto *linearSolver = l_linearSolver.get();
    linearSolver->setSystemLHVector(m_physicalGradientId);
    linearSolver->setSystemRHVector(m_geometricGradientId);
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
