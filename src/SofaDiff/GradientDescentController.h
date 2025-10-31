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
#include <SofaDiff/TrainableParameter.h>

#include <sofa/component/controller/Controller.h>
#include <sofa/core/behavior/LinearSolverAccessor.h>
#include <sofa/core/behavior/MultiVec.h>


namespace sofadiff
{

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

    SingleLink<GradientDescentController, core::State<defaulttype::Vec1dTypes>, BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_loss;
    Data<double> d_learningRate;

protected:
    core::MultiVecDerivId m_geometricGradientId;
    core::MultiVecDerivId m_physicalGradientId;
    std::vector<TrainableParameter *> m_trainableParameters;
    std::vector<ParameterizedForceField *> m_parameterizedForceFields;

    virtual void updateParameters();

private:
    void initializeLossGradientToOne();
    void resetParametersGradient() const;
    void solveForPhysicalGradient() const;
};

}
