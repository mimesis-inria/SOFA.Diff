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

#include <sofa/core/Mapping.h>


namespace sofadiff
{

/**
 * @brief CostFunctionMapping Class
 *
 * Compute a cost to minimize.
 */
template <class In>
class SOFA_SOFADIFF_API CostFunctionMapping : public core::Mapping<In, defaulttype::Vec1dTypes>
{
public:
    SOFA_ABSTRACT_CLASS(SOFA_TEMPLATE(CostFunctionMapping, In), SOFA_TEMPLATE2(core::Mapping, In, defaulttype::Vec1dTypes));
    typedef core::Mapping<In,  defaulttype::Vec1dTypes> Inherit;
    typedef defaulttype::Vec1dTypes Out;

    void init() override;
    void resetOutputGradient();
};

}
