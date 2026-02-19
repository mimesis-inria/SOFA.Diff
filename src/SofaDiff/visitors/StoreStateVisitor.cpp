#include <SofaDiff/visitors/StoreStateVisitor.h>

#include <sofa/core/behavior/BaseMechanicalState.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/VectorOperations.h>


namespace sofadiff
{
using namespace sofa::core;


StoreStateVisitor::StoreStateVisitor(const ExecParams *params, const MultiVecCoordId& positionId, const MultiVecDerivId& velocityId):
    Visitor(params),
    m_positionId(positionId),
    m_velocityId(velocityId)
{}


simulation::Visitor::Result StoreStateVisitor::processNodeTopDown(simulation::Node* node)
{
    if (!node->mechanicalState)
    {
        return RESULT_CONTINUE;
    }

    simulation::common::VectorOperations vop(params, node->getContext());
    vop.v_eq(m_positionId, vec_id::read_access::position);
    vop.v_eq(m_velocityId, vec_id::read_access::velocity);
    return RESULT_CONTINUE;
}


void StoreStateVisitor::processNodeBottomUp(simulation::Node* /*node*/)
{

}

}
