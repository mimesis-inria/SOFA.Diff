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

void GradientDescentOptimizationLoop::_updateParameters()
{
    for (auto & parameter : l_parameters)
    {
        auto nextValue = parameter->getVectorFromData("value");

        const auto learningRate = parameter->getVectorFromData("learningRate");
        const auto gradient = parameter->getVectorFromData("gradient");
        for (size_t j = 0; j < gradient.size(); j++)
            nextValue[j] -= learningRate[j] * gradient[j];

        if (parameter->hasData("lowerBound"))
        {
            const auto lowerBound = parameter->getVectorFromData("lowerBound");
            for (size_t j = 0; j < gradient.size(); j++)
                if (nextValue[j] < lowerBound[j])
                    nextValue[j] = lowerBound[j];
        }

        if (parameter->hasData("upperBound"))
        {
            const auto upperBound = parameter->getVectorFromData("upperBound");
            for (size_t j = 0; j < gradient.size(); j++)
                if (nextValue[j] > upperBound[j])
                    nextValue[j] = upperBound[j];
        }

        parameter->setDataFrom("nextValue", nextValue);
    }
}

}