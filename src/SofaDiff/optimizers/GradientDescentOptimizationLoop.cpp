#include <SofaDiff/optimizers/GradientDescentOptimizationLoop.h>

#include <sofa/core/ObjectFactory.h>


using namespace sofa::core;

namespace sofadiff
{

void registerGradientDescentOptimizationLoop(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("Gradient descent algorithm for optimization.").add< GradientDescentOptimizationLoop >());
}

void GradientDescentOptimizationLoop::updateParameters()
{
    std::vector<BaseParameter*> parameters;
    this->getContext()->get<BaseParameter>(&parameters, BaseContext::SearchDown);
    for (auto * parameter : parameters)
    {
        auto value = parameter->getValueVector();

        const auto learningRate = parameter->getHyperparameter("learningRate");
        const auto gradient = parameter->getGradientVector();
        for (size_t j = 0; j < gradient.size(); j++)
            value[j] -= learningRate * gradient[j];

        if (parameter->hasHyperparameter("lowerBound"))
        {
            const auto lowerBound = parameter->getHyperparameter("lowerBound");
            for (size_t j = 0; j < gradient.size(); j++)
                if (value[j] < lowerBound)
                    value[j] = lowerBound;
        }

        if (parameter->hasHyperparameter("upperBound"))
        {
            const auto upperBound = parameter->getHyperparameter("upperBound");
            for (size_t j = 0; j < gradient.size(); j++)
                if (value[j] > upperBound)
                    value[j] = upperBound;
        }

        parameter->setValueVector(value);
    }
}

}