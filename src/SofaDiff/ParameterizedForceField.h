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

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/MultiVec.h>


namespace sofadiff
{

/**
 *  \brief ParameterizedForceField is a ForceField that can be differentiated w.r.t. its data.
 */
class SOFA_SOFADIFF_API ParameterizedForceField//: virtual public core::objectmodel::BaseObject
{
public:
    // SOFA_ABSTRACT_CLASS(ParameterizedForceField, BaseForceField);
    // SOFA_BASE_CAST_IMPLEMENTATION(ParameterizedForceField);

    const std::vector<double> & getParameterGradient(const std::string & dataName)
    {
        // No need to check for existence of key because we check that the data exists in GradientDescentController
        return m_gradientMap[dataName];
    }

    virtual void applyParametersJacobianTranspose(const core::MechanicalParams* mparams, core::MultiVecDerivId vecId) = 0;

protected:
    std::map<std::string, std::vector<double>> m_gradientMap;
};

}
