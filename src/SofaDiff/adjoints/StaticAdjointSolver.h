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


class SOFA_SOFADIFF_API StaticAdjointSolver: public AdjointSolver, public behavior::LinearSolverAccessor
{
public:
    SOFA_CLASS2(StaticAdjointSolver, AdjointSolver, core::behavior::LinearSolverAccessor);

    void init() override;
    void resetGradients(const ExecParams *) override;
    void solve(const ExecParams* /*params*/, SReal /*dt*/, MultiVecCoordId /*xResult*/, MultiVecDerivId /*vResult*/) override;

protected:
    MultiVecDerivId m_positionGradientId;
    MultiVecDerivId m_forceGradientId;

    MultiVecDerivId & getLossGradientId() override { return m_positionGradientId; }

private:
    void initializeLossGradientToOne();
    void solveForPhysicalGradient();
};

}