#pragma once

#include <SofaDiff/config.h>

#include <sofa/core/MultiVecId.h>
#include <sofa/core/objectmodel/BaseObject.h>


namespace sofadiff
{
class SOFA_SOFADIFF_API AdjointSolver: public virtual core::objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS(AdjointSolver, sofa::core::objectmodel::BaseObject);
    // SOFA_BASE_CAST_IMPLEMENTATION(AdjointSolver)

    virtual void solve(const core::ExecParams* /*params*/, SReal /*dt*/, core::MultiVecCoordId /*xResult*/, core::MultiVecDerivId /*vResult*/) = 0;

protected:
    AdjointSolver();

    ~AdjointSolver() override;
};
}
