#include <SofaDiff/visitors/AdjointSolveVisitor.h>
#include <SofaDiff/AdjointSolver.h>

#include <sofa/simulation/Node.h>
#include <sofa/core/objectmodel/BaseContext.h>


namespace sofadiff
{

AdjointSolveVisitor::AdjointSolveVisitor(const core::ExecParams *params, SReal _dt, core::MultiVecCoordId X, core::MultiVecDerivId V):
    Visitor(params),
    dt(_dt),
    x(X),
    v(V)
{

}

simulation::Visitor::Result AdjointSolveVisitor::processNodeTopDown(simulation::Node* node)
{
    if (! node->solver.empty())
    {
        // TODO: better way to get the adjoint?
        std::vector<AdjointSolver *> adjoints;
        const auto* ctx = node->getContext();
        ctx->get<AdjointSolver> (&adjoints, core::objectmodel::BaseContext::Local); // TODO: correct search direction?
        if (adjoints.empty())
        {
            msg_error("No adjoint solver found");
            return RESULT_PRUNE;
        }
        if (adjoints.size() > 1)
        {
            msg_warning("Multiple adjoint solvers found, picking the first one"); // TODO: print name of the picked adjoint
        }
        AdjointSolver * adjoint = adjoints[0];
        adjoint->solve(params, dt, x, v); // TODO: task scheduling like SolveVisitor?
        return RESULT_PRUNE;
    }

    // TODO: What is the equivalent of what follows for the adjoint?
    // if (m_computeForceIsolatedInteractionForceFields)
    // {
    //     for_each(this, node, node->interactionForceField, &SolveVisitor::fwdInteractionForceField);
    // }
    return RESULT_CONTINUE;
}

void AdjointSolveVisitor::processNodeBottomUp(simulation::Node* /*node*/)
{

}

}
