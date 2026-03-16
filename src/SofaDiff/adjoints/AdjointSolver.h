#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/LossState.h>
#include <SofaDiff/Parameter.h>
#include <SofaDiff/ParameterizedForceField.h>

#include <sofa/core/MechanicalParams.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/core/objectmodel/BaseObject.h>


namespace sofadiff
{
using namespace sofa::core;


class SOFA_SOFADIFF_API AdjointSolver: public virtual objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS(AdjointSolver, BaseObject);

    // Implementation of the method toAdjointSolver(), assuming such a method was declared in Base.
    // Which is not the case, so it does not work. But what is the point of this method anyway?
    // SOFA_BASE_CAST_IMPLEMENTATION(AdjointSolver)

    void init() override;
    void bwdInit() override;

    // Called at the first stepAdjoint() of a DifferentiableAnimationLoop (beginning of the backpropagation)
    virtual void resetGradients(const ExecParams* /*params*/) = 0;

    // Called at each stepAdjoint() of a DifferentiableAnimationLoop
    virtual void solve(const ExecParams* /*params*/, SReal /*dt*/, MultiVecCoordId /*xResult*/, MultiVecDerivId /*vResult*/) = 0;

protected:
    std::vector<LossState *> m_lossStates;
    std::vector<BaseParameter *> m_trainableParameters;
    std::vector<Parameterized *> m_parameterizedForceFields;

    MultiVecDerivId newVecId(const char * name);
    void resetParametersGradient() const;
    void propagateGradientsThroughForceFields(const MechanicalParams * mparams, const MultiVecDerivId & forceGradientId) const;

    // Return the VecId that is used in MechanicalAccumulateVecDeriv to backpropagate the gradient of the loss
    virtual MultiVecDerivId & getLossGradientId() = 0;
};
}
