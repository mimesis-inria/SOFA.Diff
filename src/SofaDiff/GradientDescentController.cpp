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

#include <ranges>
#include <boost/parameter/parameters.hpp>
#include <SofaDiff/GradientDescentController.h>
#include <SofaDiff/ComputeCostGradientVisitor.h>
#include <SofaDiff/PropagateForceGradientVisitor.h>
#include <SofaDiff/ParameterizedForceField.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/VectorOperations.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalResetForceVisitor.h>


namespace sofadiff {

GradientDescentController::GradientDescentController()
: d_trainableParameters(initData(&d_trainableParameters, "trainableParameters", "List of links to the parameters to optimize"))
, d_learningRate(initData(&d_learningRate, "learningRate", "The learning rate for the gradient descent"))
, d_parameterGradient(initData(&d_parameterGradient, "parameterGradient", "The gradient of the parameters"))
{
    this->f_listening.setValue(true);
}

void GradientDescentController::init()
{
    LinearSolverAccessor::init();

    auto* ctx = this->getContext();
    auto* params = core::mechanicalparams::defaultInstance();

    simulation::common::VectorOperations vop(params, ctx);
    core::behavior::MultiVecDeriv vec(&vop, m_gradientVecId);
    vec.realloc(&vop, false, true, core::VecIdProperties{"Derivative of the cost w.r.t. position", this->getClassName()});
    m_gradientVecId = vec.id();

    core::behavior::MultiVecDeriv vec_2(&vop, m_forceGradientVecId);
    vec_2.realloc(&vop, false, true, core::VecIdProperties{"Derivative of the cost w.r.t. force", this->getClassName()});
    m_forceGradientVecId = vec_2.id();

    m_parametersMap = getParametersMap();
}


std::map<ParameterizedForceField *, std::vector<Data<type::vector<SReal>> *>> GradientDescentController::getParametersMap()
{
    const auto & parametersStrings = d_trainableParameters.getValue();

    std::map<ParameterizedForceField *, std::vector<Data<type::vector<SReal>> *>> parametersMap;

    for (auto& link : parametersStrings)
    {
        const size_t index = link.find_last_of('.');
        if (index >= link.size() - 1)
        {
            msg_error() << "Bad link specified: " << link;
            continue;
        }

        const std::string objectPath = link.substr(0, index);
        const std::string dataName = link.substr(index + 1);

        BaseObject::SPtr objectPtr = nullptr;
        this->getContext()->get(objectPtr, objectPath);
        if (objectPtr == nullptr)
        {
            msg_error() << "Can't find object " << objectPath;
            continue;
        }

        auto * parameterizedComponent = dynamic_cast<ParameterizedForceField*>(objectPtr.get());
        if (parameterizedComponent == nullptr)
        {
            msg_error() << "Object " << objectPath << " is not a ParameterizedForceField";
            continue;
        }

        auto * baseDataPtr = objectPtr->findData(dataName);
        if (baseDataPtr == nullptr)
        {
            msg_error() << "Can't find data " << dataName;
            continue;
        }

        auto * dataPtr = dynamic_cast<Data<type::vector<SReal>>*>(baseDataPtr);
        if (dataPtr == nullptr)
        {
            msg_error() << "Data " << dataName << " is not a Data<vector<SReal>>";
            continue;
        }

        if (!parametersMap.contains(parameterizedComponent))
            parametersMap[parameterizedComponent] = std::vector<Data<type::vector<SReal>> *> ();
        parametersMap.at(parameterizedComponent).push_back(dataPtr);
    }
    return parametersMap;
}


std::vector<double> GradientDescentController::gatherParameters()
{
    std::vector<double> parametersVector;
    for (const auto &dataVector: m_parametersMap | std::views::values)
    {
        for (const auto & data: dataVector)
        {
            for (const auto& parameters = data->getValue(); const auto& parameter : parameters)
                parametersVector.push_back(parameter);
        }
    }
    return parametersVector;
}


std::vector<double> GradientDescentController::gatherParametersGradient()
{
    std::vector<double> parametersGradientVector;
    for (const auto &[component, dataVector]: m_parametersMap)
    {
        for (const auto & data: dataVector)
        {
            const auto & derivatives = component->getParameterGradient(data->getName());
            for (const auto & derivative : derivatives)
                parametersGradientVector.push_back(derivative);
        }
    }
    return parametersGradientVector;
}


void GradientDescentController::scatterParameters(const std::vector<double> &parametersVector)
{
    unsigned int currentIndex = 0;
    for (const auto &dataVector: m_parametersMap | std::views::values)
    {
        for (const auto & data: dataVector)
        {
            const auto size = data->getValue().size();
            auto parameters = std::vector<double>(size);
            for (unsigned int i = 0; i < size; ++i)
            {
                parameters[i] = parametersVector[currentIndex + i];
            }
            currentIndex += size;
            data->setValue(parameters);
        }
    }
}



void GradientDescentController::onEndAnimationStep(const double /*dt*/)
{
    auto *ctx = this->getContext();
    auto *params = core::mechanicalparams::defaultInstance();

    // Reset the gradient data
    simulation::mechanicalvisitor::MechanicalResetForceVisitor(params, m_gradientVecId).execute(ctx, false);
    // simulation::mechanicalvisitor::MechanicalResetForceVisitor(params, m_forceGradientVecId).execute(ctx, false);

    // Compute and propagate the gradient of the cost function
    ComputeCostGradientVisitor(params, m_gradientVecId).execute(ctx, false);

    // Solve the (transpose) system to get the "force gradient"
    auto *linearSolver = l_linearSolver.get();
    linearSolver->setSystemLHVector(m_forceGradientVecId);
    linearSolver->setSystemRHVector(m_gradientVecId);
    linearSolver->solveSystem();
    simulation::common::VectorOperations vop(params, ctx);
    // vop.v_eq(m_forceGradientVecId, m_forceGradientVecId, -1.0);

    // Propagate the "force gradient" to the mapped states
    PropagateForceGradientVisitor(params, m_forceGradientVecId).execute(ctx, false);

    // Call the "applyCustomJacobianTranspose()" of the components with "trainable" parameters
    for (const auto &forceField: m_parametersMap | std::views::keys)
    {
        forceField->applyParametersJacobianTranspose(params, m_forceGradientVecId, m_parametersMap[forceField]);
    }

    // Apply the gradient: p = p - lr * grad
    const auto & parameters = gatherParameters();
    const auto & gradient = gatherParametersGradient();
    d_parameterGradient.setValue(gatherParametersGradient());
    const auto updatedParameters = getUpdatedParameters(parameters, gradient);
    scatterParameters(updatedParameters);
}


std::vector<double> GradientDescentController::getUpdatedParameters(const std::vector<double> & parameters, const std::vector<double> & gradient)
{
    auto updatedParameters = std::vector<double>(parameters.size());
    const auto learningRate = d_learningRate.getValue();
    for (unsigned int i = 0; i < parameters.size(); ++i)
    {
        updatedParameters[i] = parameters[i] - learningRate * gradient[i];
    }
    return updatedParameters;
}

void registerGradientDescentController(core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Controller that performs gradient descent.").add< GradientDescentController >());
}

}
