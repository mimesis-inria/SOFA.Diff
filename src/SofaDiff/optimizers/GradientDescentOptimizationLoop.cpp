#include <SofaDiff/optimizers/GradientDescentOptimizationLoop.h>

#include <sofa/core/ObjectFactory.h>


using namespace sofa::core;

namespace sofadiff
{

void registerGradientDescentOptimizationLoop(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Gradient descent algorithm for optimization.").add< GradientDescentOptimizationLoop >());
}


void GradientDescentOptimizationLoop::applyGradient()
{
    for (const auto parameter : m_parameters)
    {
        auto value = parameter->getValueVector();

        const auto learningRate = getHyperparameter(parameter, "learningRate");
        const auto gradient = parameter->getGradientVector();
        for (size_t j = 0; j < gradient.size(); j++)
            value[j] -= learningRate * gradient[j];

        if (hasHyperparameter(parameter, "lowerBound"))
        {
            const auto lowerBound = getHyperparameter(parameter, "lowerBound");
            for (size_t j = 0; j < gradient.size(); j++)
                if (value[j] < lowerBound)
                    value[j] = lowerBound;
        }

        if (hasHyperparameter(parameter, "upperBound"))
        {
            const auto upperBound = getHyperparameter(parameter, "upperBound");
            for (size_t j = 0; j < gradient.size(); j++)
                if (value[j] > upperBound)
                    value[j] = upperBound;
        }

        parameter->setValueVector(value);
    }
}

}