#include <SofaDiff/forcefields/ParameterizedSpringForceField.h>

#include <sofa/core/MechanicalParams.h>
#include <sofa/core/ObjectFactory.h>


namespace sofadiff
{

void ParameterizedSpringForceField::init()
{
    SpringForceField::init();
    initParameter(d_ks, m_stiffnessGradient);
    initParameter(d_lengths, m_lengthGradient);
    checkForNotImplementedParameters(this->getDataFields());
}


void ParameterizedSpringForceField::applyParametersJacobianTranspose(const core::MechanicalParams* mparams, const core::MultiVecDerivId vecId)
{
    // Skip if there is nothing to optimize
    if (m_stiffnessGradient == nullptr && m_lengthGradient == nullptr)
    {
        msg_warning() << "ParameterizedSpringForceField::applyParametersJacobianTranspose() skipped: no parameter to optimize. Consider using SpringForceField instead. Or not?";
        return;
    }

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
            if (m_stiffnessGradient != nullptr)
            {
                auto stiffnessGradient = helper::getWriteAccessor(*m_stiffnessGradient);
                stiffnessGradient[i] += elongation * dot(u, DataTypes::getDPos(v1[a]) - DataTypes::getDPos(v2[b]));
                // stiffnessGradient[i] += 0.1 * 2 * spring.ks * elongation;  // Penalization
            }
            if (m_lengthGradient != nullptr)
            {
                auto lengthGradient = helper::getWriteAccessor(*m_lengthGradient);
                lengthGradient[i] += -spring.ks * dot(u, DataTypes::getDPos(v1[a]) - DataTypes::getDPos(v2[b]));
                // lengthGradient[i] += -0.1 * 2 * spring.ks * elongation;  // Penalization
            }
        }
    }
}

void registerParameterizedSpringForceField(core::ObjectFactory* factory)
{
    factory->registerObjects(
        core::ObjectRegistrationData("Spring force field differentiable wrt stiffness.")
        .add< ParameterizedSpringForceField >());
}

}
