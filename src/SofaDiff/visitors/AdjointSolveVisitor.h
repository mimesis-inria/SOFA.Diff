#pragma once

#include <SofaDiff/config.h>

#include <sofa/core/MultiVecId.h>
#include <sofa/simulation/Visitor.h>


namespace sofadiff
{
using namespace sofa::core;


class SOFA_SOFADIFF_API AdjointSolveVisitor: public simulation::Visitor
{
public:
    AdjointSolveVisitor(const ExecParams* params, SReal _dt, const MultiVecCoordId& X, const MultiVecDerivId &V);

    Result processNodeTopDown(simulation::Node* node) override;
    void processNodeBottomUp(simulation::Node* /*node*/) override;

    /// Specify whether this action can be parallelized.
    bool isThreadSafe() const override { return false; } // TODO: I have no idea of the correct return value here

    /// Return a category name for this action.
    /// Only used for debugging / profiling purposes
    const char* getCategoryName() const override { return "behavior update position"; } // TODO: No idea
    const char* getClassName() const override { return "AdjointSolveVisitor"; }

protected:
    SReal dt;
    MultiVecCoordId x;
    MultiVecDerivId v;
};
}