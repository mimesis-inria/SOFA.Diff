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

#include <SofaDiff/ComputeCostGradientVisitor.h>

#include <sofa/core/behavior/BaseForceField.h>

namespace sofadiff
{

sofa::simulation::Visitor::Result ComputeCostGradientVisitor::fwdMechanicalState(sofa::simulation::Node* /*node*/, sofa::core::behavior::BaseMechanicalState* mm)
{
    // mm->accumulateForce(this->params, res.getId(mm));
    std::cout << "ComputeCostGradientVisitor::fwdMechanicalState" << std::endl;
    return RESULT_CONTINUE;
}


sofa::simulation::Visitor::Result ComputeCostGradientVisitor::fwdMappedMechanicalState(sofa::simulation::Node* /*node*/, sofa::core::behavior::BaseMechanicalState* mm)
{
    // mm->accumulateForce(this->params, res.getId(mm));
    return RESULT_CONTINUE;
}


sofa::simulation::Visitor::Result ComputeCostGradientVisitor::fwdForceField(sofa::simulation::Node* /*node*/, sofa::core::behavior::BaseForceField* ff)
{
    // ff->addForce(this->mparams, res);

    return RESULT_CONTINUE;
}

void ComputeCostGradientVisitor::bwdMechanicalMapping(sofa::simulation::Node* /*node*/, sofa::core::BaseMapping* map)
{
    // if (accumulate)
    // {
    //     map->applyJT(mparams, res, res);
    // }
}

void ComputeCostGradientVisitor::bwdMechanicalState(sofa::simulation::Node* , sofa::core::behavior::BaseMechanicalState* mm)
{
    SOFA_UNUSED(mm);
}

std::string ComputeCostGradientVisitor::getInfos() const
{
    std::string name=std::string("[")+res.getName()+std::string("]");
    if (accumulate) name+= " Accumulating";
    else            name+= " Not Accumulating";
    return name;
}

}
