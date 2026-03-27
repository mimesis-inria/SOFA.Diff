#include <SofaDiff/optimizers/GradientDescentOptimizationLoop.h>

#include <sofa/core/ObjectFactory.h>


using namespace sofa::core;

namespace sofadiff
{
using namespace sofa::core::objectmodel;

void registerGradientDescentOptimizationLoop(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Gradient descent algorithm for optimization.").add< GradientDescentOptimizationLoop >());
}

void GradientDescentOptimizationLoop::setParametersNextValue()
{
    for (auto & parameter : l_parameters)
    {
        auto value = parameter->getValue();

        const auto learningRate = parameter->getHyperparameter("learningRate");
        const auto gradient = parameter->getGradient();
        for (size_t j = 0; j < gradient.size(); j++)
            value[j] -= learningRate[j] * gradient[j];

        if (parameter->hasHyperparameter("lowerBound"))
        {
            const auto lowerBound = parameter->getHyperparameter("lowerBound");
            for (size_t j = 0; j < gradient.size(); j++)
                if (value[j] < lowerBound[j])
                    value[j] = lowerBound[j];
        }

        if (parameter->hasHyperparameter("upperBound"))
        {
            const auto upperBound = parameter->getHyperparameter("upperBound");
            for (size_t j = 0; j < gradient.size(); j++)
                if (value[j] > upperBound[j])
                    value[j] = upperBound[j];
        }

        parameter->setDataInGroupFromVector("nextValue", this->getName(), value);
    }
}

}