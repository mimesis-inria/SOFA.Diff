#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/Parameter.h>
#include <SofaDiff/ParameterizedForceField.h>

#include <sofa/core/MultiVecId.h>
#include <sofa/core/objectmodel/BaseObject.h>

#include "sofa/core/MechanicalParams.h"
#include "SofaDiff/visitors/AdjointResetVisitor.h"
#include "SofaDiff/visitors/AdjointResetVisitor.h"


namespace sofadiff
{
using namespace sofa::core;


class SOFA_SOFADIFF_API AdjointSolver: public virtual objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS(AdjointSolver, objectmodel::BaseObject);
    // SOFA_BASE_CAST_IMPLEMENTATION(AdjointSolver) // TODO: What's that?

    void init() override;

    virtual void resetGradients(const ExecParams* /*params*/) = 0; // TODO: could use BaseObject::reset() ?

    virtual void solve(const ExecParams* /*params*/, SReal /*dt*/, MultiVecCoordId /*xResult*/, MultiVecDerivId /*vResult*/) = 0;

protected:
    std::vector<BaseParameter *> m_trainableParameters;
    std::vector<Parameterized *> m_parameterizedForceFields;

    MultiVecDerivId newVecId(const char * name);
    void resetParametersGradient() const;
    void propagateGradientsThroughForceFields(const MechanicalParams * mparams, const MultiVecDerivId & forceGradientId) const;
};
}
