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
    void solve(const ExecParams* /*params*/, SReal /*dt*/, MultiVecCoordId /*xResult*/, MultiVecDerivId /*vResult*/) override;

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