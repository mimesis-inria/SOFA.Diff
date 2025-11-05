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
#include <SofaDiff/Parameter.h>

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/behavior/MultiVec.h>
#include <sofa/core/behavior/BaseForceField.h>


namespace sofadiff
{

class SOFA_SOFADIFF_API Parameterized: virtual public core::objectmodel::BaseObject
{
    SOFA_ABSTRACT_CLASS(Parameterized, core::objectmodel::BaseObject);

public:
    std::unordered_set<core::BaseData*> m_canBeTrained;

    virtual void applyParametersJacobianTranspose(
        const core::MechanicalParams* mparams,
        core::MultiVecDerivId vecId
    ) = 0;

    ~Parameterized() override = default;

protected:

    static BaseParameter* getParentParameter(const core::BaseData * data);

    template<class T>
    void initParameter(Data<T> & data, Data<T> * & gradientPtr);

    void checkForNotImplementedParameters(const type::vector<core::BaseData *> & dataFields) const;
};


class ParameterizedForceField: public Parameterized, virtual public core::behavior::BaseForceField
{
    const core::BaseClass* getClass() const override { return core::behavior::BaseForceField::getClass(); }
};


template<class T>
void Parameterized::initParameter(Data<T> &data, Data<T> *&gradientPtr)
{
    m_canBeTrained.insert(&data);

    auto * parameter = getParentParameter(&data);
    if (parameter == nullptr)
        return;

    auto * parameterVector = dynamic_cast<Parameter<T>*>(parameter);
    if (parameterVector == nullptr)
    {
        msg_error() << data.getName() << " parameter must be a " << Parameter<T>::GetCustomClassName();
        return;
    }

    gradientPtr = &parameterVector->d_gradient;
}

}
