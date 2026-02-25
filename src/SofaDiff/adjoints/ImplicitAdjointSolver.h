#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/adjoints/AdjointSolver.h>
#include <SofaDiff/ParameterizedForceField.h>
#include <SofaDiff/Parameter.h>
#include <SofaDiff/LossState.h>

#include <sofa/core/behavior/LinearSolverAccessor.h>


namespace sofadiff
{
using namespace sofa::core;


class SOFA_SOFADIFF_API ImplicitAdjointSolver: public AdjointSolver, public behavior::LinearSolverAccessor
{
public:
    SOFA_CLASS2(ImplicitAdjointSolver, AdjointSolver, core::behavior::LinearSolverAccessor);

    void init() override;
    void resetGradients(const ExecParams * params) override;
    void solve(const ExecParams* params, SReal dt, MultiVecCoordId /*xResult*/, MultiVecDerivId /*vResult*/) override;

protected:
    MultiVecDerivId m_positionGradientId;
    MultiVecDerivId m_velocityGradientId;
    MultiVecDerivId m_deltaVelocityGradientId;
//     std::vector<BaseParameter *> m_trainableParameters;
//     std::vector<LossState *> m_lossStates;
//
private:
    void solveForForceGradient(const ExecParams* params, SReal dt);
    void updatePositionGradient(MechanicalParams mparams, SReal dt);
    void updateVelocityGradient(MechanicalParams mparams, SReal dt);
//     void initializeLossGradientToOne();
//     void solveForPhysicalGradient();
};

}