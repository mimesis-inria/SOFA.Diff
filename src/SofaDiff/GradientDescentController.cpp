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
#include <SofaDiff/GradientDescentController.h>
#include <SofaDiff/ComputeCostGradientVisitor.h>
#include <SofaDiff/PropagateForceGradientVisitor.h>
#include <SofaDiff/ParameterizedForceField.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/VectorOperations.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalResetForceVisitor.h>


namespace sofadiff {

GradientDescentController::GradientDescentController():
    d_parameterGradient(initData(&d_parameterGradient, "parameterGradient", "The gradient of the parameters")),
    d_trainableParameters(initData(&d_trainableParameters, "trainableParameters", "List of links to the parameters to optimize"))
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

    simulation::common::VectorOperations vop2(params, ctx);
    core::behavior::MultiVecDeriv vec_2(&vop2, m_forceGradientVecId);
    vec_2.realloc(&vop2, false, true, core::VecIdProperties{"Derivative of the cost w.r.t. force", this->getClassName()});
    m_forceGradientVecId = vec_2.id();

    m_parametersMap = getObjectDataMap();
}


std::map<ParameterizedForceField*, core::objectmodel::BaseData*> GradientDescentController::getObjectDataMap()
{
    std::map<ParameterizedForceField*, core::objectmodel::BaseData*> outputMap;
    for (auto& link : d_trainableParameters.getValue())
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

        core::objectmodel::BaseData* dataPtr = objectPtr->findData(dataName);
        if (dataPtr == nullptr)
        {
            msg_error() << "Can't find data " << dataName;
            continue;
        }

        outputMap[parameterizedComponent] = dataPtr;
    }
    return outputMap;
}


void GradientDescentController::onEndAnimationStep(const double /*dt*/)
{
    std::cout << "SofaDiff::GradientDescentController::onEndAnimationStep" << std::endl;

    auto *ctx = this->getContext();
    auto *params = core::mechanicalparams::defaultInstance();

    // Reset the gradient data
    simulation::mechanicalvisitor::MechanicalResetForceVisitor(params, m_gradientVecId).execute(ctx, false);
    simulation::mechanicalvisitor::MechanicalResetForceVisitor(params, m_forceGradientVecId).execute(ctx, false);

    // Compute and propagate the gradient of the cost function
    ComputeCostGradientVisitor(params, m_gradientVecId).execute(ctx, false);

    // Solve the (transpose) system to get the "force gradient"
    auto *linearSolver = l_linearSolver.get();
    linearSolver->setSystemLHVector(m_forceGradientVecId);
    linearSolver->setSystemRHVector(m_gradientVecId);
    linearSolver->solveSystem();
    simulation::common::VectorOperations vop(params, ctx);
    vop.v_eq(m_forceGradientVecId, m_forceGradientVecId, -1.0);

    // Propagate the "force gradient" to the mapped states
    PropagateForceGradientVisitor(params, m_forceGradientVecId).execute(ctx, false);

    // Call the "applyCustomJacobianTranspose()" of the components with "trainable" parameters
    for (const auto &forceField: m_parametersMap | std::views::keys)
    {
        forceField->applyParametersJacobianTranspose(params, m_forceGradientVecId);
    }

    // TODO: Apply the gradient
}

void registerGradientDescentController(core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Controller that performs gradient descent.").add< GradientDescentController >());
}

}
