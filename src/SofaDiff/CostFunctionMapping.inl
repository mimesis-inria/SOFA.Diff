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

#include <SofaDiff/CostFunctionMapping.h>
#include <SofaDiff/GradientDescentController.h>

namespace sofadiff {

template <class TIn>
void CostFunctionMapping<TIn>::init()
{
    Inherit::init();
}

template<class In>
void CostFunctionMapping<In>::resetOutputGradient()
{
    if(auto * controller = this->getContext()->getRootContext()->template get<GradientDescentController>())
    {
        auto * output = dynamic_cast<core::behavior::MechanicalState<defaulttype::Vec1Types> *> (Inherit::getTo()[0]);
        const core::MultiVecDerivId multi_gradient = controller->getGradientVecId();
        const auto& gradient = multi_gradient.getId(output);
        helper::WriteAccessor<Data<VecDeriv_t<defaulttype::Vec1Types>> > outputGradient = output->write(gradient);
        outputGradient[0] = sofa::Deriv_t<defaulttype::Vec1Types> (1);
    }
}


}
