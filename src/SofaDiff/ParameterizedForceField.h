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

#include <unordered_set>

#include <SofaDiff/config.h>
#include <SofaDiff/TrainableParameter.h>

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/MultiVec.h>
#include <sofa/core/behavior/BaseForceField.h>


namespace sofadiff
{

class SOFA_SOFADIFF_API Parameterized
{
public:
    std::unordered_set<core::BaseData*> m_canBeTrained;

    virtual void applyParametersJacobianTranspose(
        const core::MechanicalParams* mparams,
        core::MultiVecDerivId vecId
    ) = 0;

    virtual ~Parameterized() = default;

protected:

    static TrainableParameter* getParentParameter(const core::BaseData * data);

    void initParameter(Data<type::vector<SReal>> & data, Data<type::vector<SReal>> * & gradientPtr);
    void initParameter(Data<SReal> & data, Data<SReal> * & gradientPtr);

    void checkForNotImplementedParameters(const type::vector<core::BaseData *> & dataFields) const;
};


// TODO: Inherit ForceField<> rather than BaseForceField?
class ParameterizedForceField: public Parameterized, public core::behavior::BaseForceField
{};

}
