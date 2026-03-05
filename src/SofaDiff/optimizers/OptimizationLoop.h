#pragma once

#include <SofaDiff/config.h>
#include <SofaDiff/Parameter.h>

#include <sofa/core/behavior/BaseAnimationLoop.h>


namespace sofadiff
{

class SOFA_SOFADIFF_API OptimizationLoop: public core::behavior::BaseAnimationLoop
{
public:
    SOFA_ABSTRACT_CLASS(OptimizationLoop, core::behavior::BaseAnimationLoop);

    OptimizationLoop();

    void init() override;
    void bwdInit() override;

    virtual bool isUpdateReady() { return true; }
    virtual void updateParameters() = 0;
    void step(const core::ExecParams *params, SReal dt) override;

protected:
    Data<int> m_totalTimesteps;
    BaseAnimationLoop * m_animationLoop;
    std::vector<BaseParameter *> m_parameters;

    static bool hasHyperparameter(const BaseParameter *parameter, const std::string &hyperparameterName) ;
    SReal getHyperparameter(const BaseParameter *parameter, const std::string &hyperparameterName) const;
};

}
