#include <SofaDiff/visitors/RetrieveStateVisitor.h>

#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/VectorOperations.h>


namespace sofadiff
{
using namespace sofa::core;


RetrieveStateVisitor::RetrieveStateVisitor(const ExecParams *params, const MultiVecCoordId& positionId, const MultiVecDerivId& velocityId):
    Visitor(params),
    m_positionId(positionId),
    m_velocityId(velocityId)
{}


simulation::Visitor::Result RetrieveStateVisitor::processNodeTopDown(simulation::Node* node)
{
    if (!node->mechanicalState)
    {
        return RESULT_CONTINUE;
    }

    simulation::common::VectorOperations vop(params, node->getContext());
    vop.v_eq(vec_id::write_access::position, m_positionId);
    vop.v_eq(vec_id::write_access::velocity, m_velocityId);
    return RESULT_CONTINUE;
}


void RetrieveStateVisitor::processNodeBottomUp(simulation::Node* /*node*/)
{

}

}
