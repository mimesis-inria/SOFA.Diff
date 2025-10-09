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

    /**
     * @brief Initialize the VecIds for computing and storing the derivatives
     */
    GradientDescentController();

    void init() override;

    /**
     * @brief Compute the derivatives
     */
    void onEndAnimationStep(double dt) override;

    core::MultiVecDerivId getGradientVecId() { return m_gradientVecId; }
    core::MultiVecDerivId getForceGradientVecId() { return m_forceGradientVecId; }

private:
    /** VecId for the derivative of the loss w.r.t. the position */
    core::MultiVecDerivId m_gradientVecId;

    /** VecId for the derivative of the loss w.r.t. the force */
    core::MultiVecDerivId m_forceGradientVecId;

    Data<type::vector<std::string>> d_trainableParameters;

    std::map<BaseObject::SPtr, core::objectmodel::BaseData*> getObjectDataMap();

    std::map<BaseObject::SPtr, core::objectmodel::BaseData*> m_parametersMap;
};

}
