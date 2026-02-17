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
#include <SofaDiff/DifferentiableAnimationLoop.h>
#include <SofaDiff/visitors/MechanicalAccumulateVecDeriv.h>
#include <SofaDiff/visitors/MechanicalPropagateVecDeriv.h>
#include <SofaDiff/ParameterizedForceField.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalResetForceVisitor.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/MechanicalOperations.h>


namespace sofadiff {

using namespace simulation::mechanicalvisitor;


GradientDescentController::GradientDescentController()
{
    this->f_listening.setValue(true); // TODO: Why?
}


void GradientDescentController::init()
{
    LinearSolverAccessor::init();

    const auto* ctx = this->getContext();
    ctx->get<BaseParameter> (&m_trainableParameters, BaseContext::SearchRoot);
}


void GradientDescentController::onEndAnimationStep(const double /*dt*/)
{
    SCOPED_TIMER("GradientDescentController::Solve");
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


void registerGradientDescentController(core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Controller that performs gradient descent.").add< GradientDescentController >());
}


}
