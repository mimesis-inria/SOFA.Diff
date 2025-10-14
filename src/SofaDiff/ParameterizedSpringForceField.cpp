#include <SofaDiff/ParameterizedSpringForceField.h>

#include <sofa/core/ObjectFactory.h>

#include "sofa/core/MechanicalParams.h"


namespace sofadiff
{

void ParameterizedSpringForceField::applyParametersJacobianTranspose(const core::MechanicalParams* mparams, const core::MultiVecDerivId vecId)
{
    std::cout << "ParameterizedSpringForceField::applyParametersJacobianTranspose" << std::endl;
    m_derivative = 0.0;

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
    for (const auto & spring : _springs)
    {
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
            m_derivative += elongation * dot(u, DataTypes::getDPos(v1[a]) - DataTypes::getDPos(v2[b]));
        }
    }

    std::cout << "Derivative:" << m_derivative << std::endl;
}

void registerParameterizedSpringForceField(core::ObjectFactory* factory)
{
    factory->registerObjects(
        core::ObjectRegistrationData("Spring force field differentiable wrt stiffness.")
        .add< ParameterizedSpringForceField >());
}

}
