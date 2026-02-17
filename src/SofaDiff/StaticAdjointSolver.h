#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/AdjointSolver.h>
#include <SofaDiff/ParameterizedForceField.h>
#include <SofaDiff/Parameter.h>
#include <SofaDiff/LossState.h>

#include <sofa/core/behavior/LinearSolverAccessor.h>


namespace sofadiff
{
class SOFA_SOFADIFF_API StaticAdjointSolver: public AdjointSolver, public core::behavior::LinearSolverAccessor
{
public:
    SOFA_CLASS2(StaticAdjointSolver, AdjointSolver, core::behavior::LinearSolverAccessor);

    StaticAdjointSolver();

    void init() override;

    void solve(const core::ExecParams* /*params*/, SReal /*dt*/, core::MultiVecCoordId /*xResult*/, core::MultiVecDerivId /*vResult*/) override;

protected:
    std::vector<BaseParameter *> m_trainableParameters;
    std::vector<Parameterized *> m_parameterizedForceFields;
    std::vector<LossState *> m_lossStates;

private:
    void initializeLossGradientToOne();
    void resetParametersGradient() const;
    void solveForPhysicalGradient();
};

}