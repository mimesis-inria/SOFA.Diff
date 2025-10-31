#include <SofaDiff/forcefields/ParameterizedSpringForceField.h>

#include <sofa/core/MechanicalParams.h>
#include <sofa/core/ObjectFactory.h>

#include "SofaDiff/TrainableParameter.h"


namespace sofadiff
{

void ParameterizedSpringForceField::init()
{
    SpringForceField::init();
    m_canBeTrained = getCanBeTrained();
    for (const auto& data : this->getDataFields())
    {
        if (!isTrainableParameter(data))
            continue;

        if (!m_canBeTrained[data])
        {
            msg_error() << "Data " << data->getName() << " is not trainable.";
            continue;
        }

        m_isTrained[data] = true;
    }
}

std::map<core::BaseData *, bool> ParameterizedSpringForceField::getCanBeTrained()
{
    std::map<core::BaseData *, bool> canBeTrained;
    for (const auto& data : this->getDataFields())
        canBeTrained[data] = false;
    canBeTrained[static_cast<core::BaseData*>(&d_ks)] = true;
    canBeTrained[static_cast<core::BaseData*>(&d_lengths)] = true;
    return canBeTrained;
}


void ParameterizedSpringForceField::applyParametersJacobianTranspose(const core::MechanicalParams* mparams, const core::MultiVecDerivId vecId)
{
    // Skip if there is nothing to optimize
    if (std::ranges::all_of(m_isTrained, [](const auto& p) { return p.second == false; }))
    {
        msg_warning() << "ParameterizedSpringForceField::applyParametersJacobianTranspose() skipped: no parameter to optimize. Consider using SpringForceField instead. Or not?";
        return;
    }

    std::vector<double> stiffnessGradient = {};
    std::vector<double> lengthGradient = {};

    // Get the mstates involved
    const auto state1 = this->mstate1.get(); // From PairInteractionForceField::addForce()
    const auto state2 = this->mstate2.get();
    if (!state1 || !state2)
    {
        msg_error() << "ParameterizedSpringForceField::applyParametersJacobianTranspose(const MechanicalParams* mparams, MultiVecDerivId vecId ), mstate missing";
        return;
    }

    // Get the vectors to multiply with
    const auto & data_v1 = *vecId[state1].read(); // Adapted from PairInteractionForceField::addForce()
    const auto & data_v2 = *vecId[state2].read();

    const helper::ReadAccessor<Data<VecDeriv>> v1 = helper::getReadAccessor(data_v1);
    const helper::ReadAccessor<Data<VecDeriv>> v2 = helper::getReadAccessor(data_v2);

    // Get the position of the mstates
    const auto x1 = mparams->readX(state1)->getValue(); // Adapted from PairInteractionForceField::addForce() and SpringForceField::addForce()
    const auto x2 = mparams->readX(state2)->getValue();

    const type::vector<Spring>& _springs = this->d_springs.getValue(); // Adapted from SpringForceField::addForce()
    for (unsigned int i=0; i < _springs.size(); i++)
    {
        const Spring spring = _springs[i];
        const Index a = spring.m1;
        const Index b = spring.m2;

        DataTypes::CPos u = DataTypes::getCPos(x2[b])-DataTypes::getCPos(x1[a]);
        const Real d = u.norm();
        if(spring.enabled && d>1.0e-9 && (!spring.elongationOnly || d>spring.initpos))
        {
            // F =   k_s.(l-l_0 ).U + k_d((V_b - V_a).U).U = f.U   where f is the intensity and U the direction
            // dF/dk_s = (l-l_0).U
            // (dF/dk_s)^T @ v = (l-l_0).dot(U, v)
            u /= d;
            const Real elongation = d - spring.initpos;
            // TODO: do only the required computations (may require to improve on TrainableParameter’s interface, in particular to be able to do gradient[i] += xxx)
            const auto grad = elongation * dot(u, DataTypes::getDPos(v1[a]) - DataTypes::getDPos(v2[b]));
            stiffnessGradient.push_back(grad);
            const auto lengthGrad = -spring.ks * dot(u, DataTypes::getDPos(v1[a]) - DataTypes::getDPos(v2[b]));
            lengthGradient.push_back(lengthGrad);
        }
        else
        {
            stiffnessGradient.push_back(0.0);
            lengthGradient.push_back(0.0);
        }
    }
    // TODO: remove this (cf TODO above)
    if (m_isTrained[static_cast<core::BaseData*>(&d_ks)])
        addToDataGradient(d_ks, stiffnessGradient);
    if (m_isTrained[static_cast<core::BaseData*>(&d_lengths)])
        addToDataGradient(d_lengths, lengthGradient);
}

void registerParameterizedSpringForceField(core::ObjectFactory* factory)
{
    factory->registerObjects(
        core::ObjectRegistrationData("Spring force field differentiable wrt stiffness.")
        .add< ParameterizedSpringForceField >());
}

}
