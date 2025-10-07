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

#include <SofaDiff/GradientDescentController.h>
#include <SofaDiff/ComputeCostGradientVisitor.h>
#include <SofaDiff/PropagateForceGradientVisitor.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/VectorOperations.h>


namespace sofadiff {

GradientDescentController::GradientDescentController()
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
}


void GradientDescentController::onEndAnimationStep(const double /*dt*/)
{
    std::cout << "SofaDiff::GradientDescentController::onEndAnimationStep" << std::endl;

    // Compute and propagate the gradient of the cost function
    auto* ctx = this->getContext();
    auto* params = core::mechanicalparams::defaultInstance();
    ComputeCostGradientVisitor(params, m_gradientVecId).execute(ctx, false);

    // Solve the (transpose) system to get the "force gradient"
    auto* linearSolver = l_linearSolver.get();
    linearSolver->setSystemLHVector(m_forceGradientVecId);
    linearSolver->setSystemRHVector(m_gradientVecId);
    linearSolver->solveSystem();
    simulation::common::VectorOperations vop(params, ctx);
    vop.v_eq(m_forceGradientVecId, m_forceGradientVecId, -1.0);

    // Propagate the "force gradient" to the mapped states
    PropagateForceGradientVisitor(params, m_forceGradientVecId).execute(ctx, false);

    // TODO: Call the "applyJT" of the components with "trainable" parameters
}

void registerGradientDescentController(core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Controller that performs gradient descent.").add< GradientDescentController >());
}

}
