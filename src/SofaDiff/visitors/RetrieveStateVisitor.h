#pragma once

#include <SofaDiff/config.h>

#include <sofa/core/MultiVecId.h>
#include <sofa/simulation/Visitor.h>
#include <sofa/simulation/Node.h>


namespace sofadiff
{
using namespace sofa::core;


class SOFA_SOFADIFF_API RetrieveStateVisitor: public simulation::Visitor
{
public:
    RetrieveStateVisitor(const ExecParams* params, const MultiVecCoordId& positionId, const MultiVecDerivId& velocityId);

    Result processNodeTopDown(simulation::Node* node) override;
    void processNodeBottomUp(simulation::Node* /*node*/) override;

    /// Specify whether this action can be parallelized.
    bool isThreadSafe() const override { return false; } // TODO: I have no idea of the correct return value here

    /// Return a category name for this action.
    /// Only used for debugging / profiling purposes
    const char* getCategoryName() const override { return "behavior update position"; } // TODO: No idea
    const char* getClassName() const override { return "StoreStateVisitor"; }

protected:
    MultiVecCoordId m_positionId;
    MultiVecDerivId m_velocityId;
};

}