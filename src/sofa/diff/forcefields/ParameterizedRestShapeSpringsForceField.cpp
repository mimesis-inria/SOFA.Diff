/******************************************************************************
*                                 SOFA.Diff                                   *
*                              (c) 2026 INRIA                                 *
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
* Authors: Léo Bois, Paul Baksic                                              *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/diff/forcefields/ParameterizedRestShapeSpringsForceField.h>

#include <sofa/core/MechanicalParams.h>
#include <sofa/core/ObjectFactory.h>


namespace sofadiff
{

void ParameterizedRestShapeSpringsForceField::init()
{
    RestShapeSpringsForceField::init();
    m_stiffnessParameter = initParameter(d_stiffness);
    checkForNotImplementedParameters(this->getDataFields());
}


void ParameterizedRestShapeSpringsForceField::applyParametersJacobianTranspose(const core::MechanicalParams* mparams, const core::MultiVecDerivId vecId)
{
    // Skip if there is nothing to optimize
    if (m_stiffnessParameter == nullptr)
    {
        msg_warning() << "ParameterizedRestShapeSpringsForceField::applyParametersJacobianTranspose() skipped: no parameter to optimize. Consider using ParameterizedRestShapeSpringsForceField instead. Or not?";
        return;
    }

    // Get the state position
    const auto state = this->mstate.get();
    if (!state)
    {
        msg_error() << "applyParametersJacobianTranspose(): mstate missing";
        this->d_componentState.setValue(core::objectmodel::ComponentState::Invalid);
        return;
    }
    const helper::ReadAccessor< DataVecCoord > p1 = state->readPositions();

    // Get the external position
    const DataVecCoord* extPosition = this->getExtPosition();
    if (!extPosition)
    {
        msg_error() << "applyParametersJacobianTranspose(): getExtPosition() failed";
        this->d_componentState.setValue(core::objectmodel::ComponentState::Invalid);
        return;
    }
    const helper::ReadAccessor< DataVecCoord > p0 = *extPosition;

    // Get the vector to multiply with
    const auto & data_v = *vecId[state].read();
    const helper::ReadAccessor<Data<VecDeriv>> v = helper::getReadAccessor(data_v);
    if (d_recompute_indices.getValue())
    {
        recomputeIndices();
    }

    for (Index i = 0; i < m_indices.size(); i++)
    {
        const Index index = m_indices[i];
        const Index ext_index = m_indices[i];
        const auto activeDirections = d_activeDirections.getValue();

        // rigid case
        if constexpr (sofa::type::isRigidType<DataTypes>)
        {
            // Maybe later
        }
        else // non-rigid implementation
        {
            Deriv dx = p1[index] - p0[ext_index];
            for (Size entryId = 0; entryId < spatial_dimensions; ++entryId)
            {
                if (!activeDirections[entryId])
                    dx[entryId] = 0;
            }
            auto stiffnessGradient = helper::getWriteAccessor(m_stiffnessParameter->d_gradient);
            stiffnessGradient[i] -= dot(dx, DataTypes::getDPos(v[index]));
        }
    }
}

void registerParameterizedRestShapeSpringsForceField(core::ObjectFactory* factory)
{
    factory->registerObjects(
        core::ObjectRegistrationData("RestShapeSprings force field differentiable wrt stiffness.")
        .add< ParameterizedRestShapeSpringsForceField >());
}

}
