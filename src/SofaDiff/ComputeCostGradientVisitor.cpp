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
#include <SofaDiff/CostFunctionMapping.h>

#include <sofa/core/trait/DataTypes.h>
#include <sofa/core/VecId.h>

namespace sofadiff
{

void ComputeCostGradientVisitor::bwdMechanicalMapping(simulation::Node * /* node */, core::BaseMapping *map)
{
    // TODO: use custom Id (e.g. "gradient") instead of force
    const core::VecDerivId result = core::vec_id::write_access::force;
    // if map is a CostFunctionMapping, then set the "gradient" of the output to 1
    if (dynamic_cast<CostFunctionMapping<defaulttype::Vec3Types> *>(map)
        or dynamic_cast<CostFunctionMapping<defaulttype::Vec2Types> *>(map)
        or dynamic_cast<CostFunctionMapping<defaulttype::Vec6Types> *>(map)
        or dynamic_cast<CostFunctionMapping<defaulttype::Rigid3Types> *>(map)
        or dynamic_cast<CostFunctionMapping<defaulttype::Rigid2Types> *>(map))
    {
        auto * output = dynamic_cast<core::behavior::MechanicalState< defaulttype::Vec1Types> * > (map->getTo()[0]);
        helper::WriteAccessor<Data<VecDeriv_t<defaulttype::Vec1Types>> > outputGradient = output->write(result);
        outputGradient[0] = sofa::Deriv_t<defaulttype::Vec1Types> (1);
    }
    // propagate gradient with applyJT() (does nothing if "gradient" is not in the output of the mapping)
    map->applyJT(mparams, result, result);
}

std::string ComputeCostGradientVisitor::getInfos() const
{
    auto name = std::string("[xxx]");
    return name;
}

}
