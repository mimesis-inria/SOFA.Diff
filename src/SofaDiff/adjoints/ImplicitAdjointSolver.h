#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/adjoints/AdjointSolver.h>
#include <SofaDiff/ParameterizedForceField.h>

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
    MultiVecDerivId m_lossGradientId;
    MultiVecDerivId m_positionGradientId;
    MultiVecDerivId m_velocityGradientId;
    MultiVecDerivId m_deltaVelocityGradientId;
    MultiVecDerivId m_residualGradientId;
    MultiVecDerivId m_forceGradientId;

    MultiVecDerivId & getLossGradientId() override { return m_lossGradientId; }

private:
    void solveForResidualGradient(const ExecParams* params, SReal dt);
    void addMatrixVectorProduct(const MultiVecDerivId& outVectorId, const MultiVecDerivId &inVectorId, SReal kFact, SReal bFact);
};

}