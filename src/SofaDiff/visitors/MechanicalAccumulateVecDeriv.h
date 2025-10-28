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

#include <sofa/simulation/MechanicalVisitor.h>

namespace sofadiff
{

// TODO: rename as AccumulateVecDerivVisitor?
/** Compute the derivative of the loss w.r.t. the position
*/
class SOFA_SOFADIFF_API MechanicalAccumulateVecDeriv : public simulation::MechanicalVisitor
{
public:
    explicit MechanicalAccumulateVecDeriv(const core::MechanicalParams* params, const core::MultiVecDerivId& vec_id)
            : MechanicalVisitor(params), m_vec_id(vec_id) {}

    /// Compute the gradient of the cost function w.r.t. the degrees of freedom
    void bwdMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* map) override;

    bool stopAtMechanicalMapping(simulation::Node* /*node*/, sofa::core::BaseMapping* /*map*/) override
    {
        return false; // !map->isMechanical();
    }

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    [[nodiscard]] const char* getClassName() const override {return "ComputeCostGradientVisitor";}
    [[nodiscard]] std::string getInfos() const override;

    /// Specify whether this action can be parallelized.
    [[nodiscard]] bool isThreadSafe() const override
    {
        return true;
    }

private:
    core::MultiVecDerivId m_vec_id;
};


}
