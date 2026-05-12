/******************************************************************************
*                                 SOFA.Diff                                   *
*                              (c) 2026 INRIA                                 *
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
* Authors: Léo Bois, Paul Baksic                                              *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/diff/adjoints/StaticAdjointSolver.h>
#include <sofa/diff/DifferentiableAnimationLoop.h>
#include <sofa/diff/visitors/MechanicalAccumulateVecDeriv.h>
#include <sofa/diff/visitors/MechanicalPropagateVecDeriv.h>

#include <sofa/core/MechanicalParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalResetForceVisitor.h>


namespace sofadiff
{
using namespace sofa::simulation::mechanicalvisitor;
using namespace sofa::core;


void registerStaticAdjointSolver(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Adjoint solver for static problems.").add< StaticAdjointSolver >());
}

void StaticAdjointSolver::init()
{
    AdjointSolver::init();
    LinearSolverAccessor::init();

    m_positionGradientId = newVecId("gradient of the global loss wrt x");
    m_forceGradientId = newVecId("gradient of the global loss wrt f");
}

void StaticAdjointSolver::resetGradients(const ExecParams * params)
{
    auto *ctx = this->getContext()->getRootContext(); // TODO: maybe not the root context? But needs access to loss and parameters...
    const MechanicalParams mparams(*params);

    MechanicalResetForceVisitor(&mparams, m_positionGradientId).execute(ctx, false);
    resetParametersGradient();
}


void StaticAdjointSolver::solve(const ExecParams* params, SReal /*dt*/, MultiVecCoordId /*xResult*/, MultiVecDerivId /*vResult*/)
{
    auto *ctx = this->getContext()->getRootContext(); // TODO: maybe not the root context? But needs access to loss and parameters...
    const MechanicalParams mparams(*params);

    MechanicalAccumulateVecDeriv(&mparams, m_positionGradientId).execute(ctx, false);
    solveForPhysicalGradient();
    MechanicalPropagateVecDeriv(&mparams, m_forceGradientId).execute(ctx, false);
    propagateGradientsThroughForceFields(&mparams, m_forceGradientId);
}

void StaticAdjointSolver::solveForPhysicalGradient()
{
    SCOPED_TIMER("StaticAdjointSolver::solveForPhysicalGradient");
    auto *linearSolver = l_linearSolver.get();

    const auto * params = execparams::defaultInstance();
    simulation::common::MechanicalOperations mop(params, this->getContext());
    mop->setImplicit(true);
    static constexpr MatricesFactors::M m(0);
    static constexpr MatricesFactors::B b(0);
    static constexpr MatricesFactors::K k(-1);
    mop.setSystemMBKMatrix(m, b, k, linearSolver);

    linearSolver->getLinearSystem()->setSystemSolution(m_forceGradientId);
    linearSolver->getLinearSystem()->setRHS(m_positionGradientId);
    linearSolver->solveSystem();  // Solve -df/dx * lambda = dy/dx
    linearSolver->getLinearSystem()->dispatchSystemSolution(m_forceGradientId);

    mop.projectResponse(m_forceGradientId);  // Take the projective constraints into account
}

}