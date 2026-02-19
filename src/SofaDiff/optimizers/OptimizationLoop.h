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

    void init() override;
    void bwdInit() override;

    void step(const core::ExecParams *params, SReal dt) override;

protected:
    BaseAnimationLoop * m_animationLoop;
    std::vector<BaseParameter *> m_parameters;

    virtual void updateParameters() = 0;
    SReal getHyperparameter(const BaseParameter *parameter, const std::string &hyperparameterName) const;


};

}
