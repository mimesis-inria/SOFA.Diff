#include <SofaDiff/ParameterizedSpringForceField.h>

#include <sofa/core/ObjectFactory.h>


namespace sofadiff
{

void ParameterizedSpringForceField::addForce(const core::MechanicalParams* mparams, core::MultiVecDerivId fId)
{
    std::cout << "ParameterizedSpringForceField::addForce" << std::endl;
}

void ParameterizedSpringForceField::addDForce(const core::MechanicalParams* mparams, core::MultiVecDerivId dfId)
{
    std::cout << "ParameterizedSpringForceField::addDForce" << std::endl;
}

void ParameterizedSpringForceField::addKToMatrix(const core::MechanicalParams* mparams, const core::behavior::MultiMatrixAccessor* matrix)
{
    std::cout << "ParameterizedSpringForceField::addKToMatrix" << std::endl;
}

SReal ParameterizedSpringForceField::getPotentialEnergy(const core::MechanicalParams* mparams) const
{
    msg_error() << "ParameterizedSpringForceField::getPotentialEnergy-not-implemented !!!";
    return 0;
}

void ParameterizedSpringForceField::applyParametersJacobianTranspose()
{
    std::cout << "ParameterizedSpringForceField::applyParametersJacobianTranspose" << std::endl;
}

void registerParameterizedSpringForceField(core::ObjectFactory* factory)
{
    factory->registerObjects(
        core::ObjectRegistrationData("Spring force field differentiable wrt stiffness.")
        .add< ParameterizedSpringForceField >());
}

}