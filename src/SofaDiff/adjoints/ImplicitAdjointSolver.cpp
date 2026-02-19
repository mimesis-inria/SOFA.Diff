#include <SofaDiff/adjoints/ImplicitAdjointSolver.h>

#include <sofa/core/ObjectFactory.h>


namespace sofadiff
{
using namespace sofa::core;


void registerImplicitAdjointSolver(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Adjoint solver for implicit Euler.").add< ImplicitAdjointSolver >());
}

void ImplicitAdjointSolver::init()
{

}

void ImplicitAdjointSolver::solve(const ExecParams *, SReal, MultiVecCoordId, MultiVecDerivId)
{
    // set LossState gradient to the derivative of the total loss wrt the current instant loss
    // yn.grad = dy/dyn

    // propagate said gradient to the main dofs (position & velocity) through the mappings
    // this gradient is added to the main dofs gradients, as it should be
    // xn.grad += yn.grad * dyn/dxn
    // vn.grad += yn.grad * dyn/dvn
    // ?? pn.grad += yn.grad * dyn/dpn ??

    // solve the system for the "force gradient"
    // F(dv; xn, vn, pn) = 0 --> f.grad * dF/d(dv) = - dv.grad = - vn.grad - dt * xn.grad

    // propagate the force gradient to the mapped states through the mappings

    // propagate the "force gradient" to the parameters through the ParameterizedForceFields
    // ?? pn.grad += f.grad * df/dp

    // update the "dofs gradient" by propagating the "force gradient"
    // this gradient replaces the previous one
    // xn.grad = f.grad * df/dx
    // vn.grad = f.grad * df/dv
}

}