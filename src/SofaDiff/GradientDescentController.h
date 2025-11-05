/******************************************************************************
 *                 SOFA, Simulation Open-Framework Architecture                *
 *                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
 *                                                                             *
 * This program is free software; you can redistribute it and/or modify it     *
 * under the terms of the GNU Lesser General Public License as published by    *
 * the Free Software Foundation; either version 2.1 of the License, or (at     *
 * your option) any later version.                                             *
 *                                                                             *
 * This program is distributed in the hope that it will be useful, but WITHOUT *
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
 * for more details.                                                           *
 *                                                                             *
 * You should have received a copy of the GNU Lesser General Public License    *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.        *
 *******************************************************************************
 * Authors: The SOFA Team and external contributors (see Authors.txt)          *
 *                                                                             *
 * Contact information: contact@sofa-framework.org                             *
 ******************************************************************************/
#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/ParameterizedForceField.h>
#include <SofaDiff/Parameter.h>

#include <sofa/component/controller/Controller.h>
#include <sofa/core/behavior/LinearSolverAccessor.h>
#include <sofa/core/behavior/MultiVec.h>

#include "LossState.h"


namespace sofadiff
{

extern core::MultiVecDerivId s_geometricGradientId;
extern core::MultiVecDerivId s_PhysicalGradientId;

/**
 * @brief GradientDescentController Class
 *
 * Performs a step of gradient descent at the end of each time step.
 */
class SOFA_SOFADIFF_API GradientDescentController :
    public component::controller::Controller,
    public core::behavior::LinearSolverAccessor
{
public:
    SOFA_CLASS2(GradientDescentController, sofa::component::controller::Controller, core::behavior::LinearSolverAccessor);

    GradientDescentController();
    void init() override;
    void onEndAnimationStep(double dt) override;

    static constexpr std::array<std::string, 1> getHyperparametersNames() { return {"learningRate"}; }

protected:
    std::vector<BaseParameter *> m_trainableParameters;
    std::vector<Parameterized *> m_parameterizedForceFields;
    std::vector<LossState *> m_lossStates;

    SReal getHyperparameter(const BaseParameter *parameter, const std::string &hyperparameterName) const;
    virtual void updateParameters();

private:
    void initializeLossGradientToOne();
    void resetParametersGradient() const;
    void solveForPhysicalGradient() const;
};

}
