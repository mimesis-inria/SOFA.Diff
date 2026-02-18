#pragma once

#include <SofaDiff/config.h>

#include <sofa/core/MultiVecId.h>
#include <sofa/core/objectmodel/BaseObject.h>


namespace sofadiff
{
using namespace sofa::core;


class SOFA_SOFADIFF_API AdjointSolver: public virtual objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS(AdjointSolver, objectmodel::BaseObject);
    // SOFA_BASE_CAST_IMPLEMENTATION(AdjointSolver) // TODO: What's that?

    virtual void solve(const ExecParams* /*params*/, SReal /*dt*/, MultiVecCoordId /*xResult*/, MultiVecDerivId /*vResult*/) = 0;
};
}
