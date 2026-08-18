/******************************************************************************
*                                 SOFA.Diff                                   *
*                              (c) 2026 INRIA                                 *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: Léo Bois, Paul Baksic                                              *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/diff/config.h>
#include <sofa/diff/initSofaDiff.h>

#include <sofa/core/ObjectFactory.h>
using sofa::core::ObjectFactory;
#include <sofa/helper/system/PluginManager.h>

namespace sofadiff
{
    extern void registerTrainableParameter(sofa::core::ObjectFactory* factory);
    extern void registerLossState(sofa::core::ObjectFactory* factory);

    extern void registerGridSearchOptimizationLoop(sofa::core::ObjectFactory* factory);
    extern void registerGradientDescentOptimizationLoop(sofa::core::ObjectFactory* factory);
    extern void registerDifferentiableAnimationLoop(sofa::core::ObjectFactory* factory);
    extern void registerStaticAdjointSolver(sofa::core::ObjectFactory* factory);
    extern void registerParameterizedRestShapeSpringsForceField(sofa::core::ObjectFactory* factory);
    extern void registerParameterizedTetrahedronFEMForceField(sofa::core::ObjectFactory* factory);
    extern void registerParameterizedBeamFEMForceField(sofa::core::ObjectFactory* factory);
    extern void registerParameterizedHeterogeneousBeamFEMForceField(sofa::core::ObjectFactory* factory);
    extern void registerParameterizedConstantForceField(sofa::core::ObjectFactory* factory);
    
    extern void registerParameterizedFixedProjectiveConstraint(sofa::core::ObjectFactory* factory);

    extern void registerMeanSquaredErrorMapping(sofa::core::ObjectFactory* factory);
    extern void registerMeanSquaredErrorMultiMapping(sofa::core::ObjectFactory* factory);
    extern void registerGeodesicPoseLossMapping(sofa::core::ObjectFactory* factory);

    extern void registerLoopsControlsGUI();


    extern "C" {
        SOFA_SOFADIFF_API void initExternalModule();
        SOFA_SOFADIFF_API const char* getModuleName();
        SOFA_SOFADIFF_API const char* getModuleVersion();
        SOFA_SOFADIFF_API const char* getModuleLicense();
        SOFA_SOFADIFF_API const char* getModuleDescription();
        SOFA_SOFADIFF_API void registerObjects(sofa::core::ObjectFactory* factory);
    }

    void initExternalModule()
    {
        init();
    }

    void init()
    {
        static bool first = true;
        if (first)
        {
            // make sure that this plugin is registered into the PluginManager
            sofa::helper::system::PluginManager::getInstance().registerPlugin(MODULE_NAME);

            first = false;
        }
    }

    const char* getModuleName()
    {
        return MODULE_NAME;
    }

    const char* getModuleVersion()
    {
        return MODULE_VERSION;
    }

    const char* getModuleLicense()
    {
        return "LGPL";
    }

    const char* getModuleDescription()
    {
        return "A plugin for differentiable physics.";
    }

    void registerObjects(sofa::core::ObjectFactory* factory)
    {
        registerTrainableParameter(factory);
        registerLossState(factory);
        
        registerGridSearchOptimizationLoop(factory);
        registerGradientDescentOptimizationLoop(factory);
        registerDifferentiableAnimationLoop(factory);

        registerStaticAdjointSolver(factory);
        registerParameterizedSpringForceField(factory);
        registerParameterizedRestShapeSpringsForceField(factory);
        registerParameterizedTetrahedronFEMForceField(factory);
        registerParameterizedBeamFEMForceField(factory);
        registerParameterizedHeterogeneousBeamFEMForceField(factory);
        registerParameterizedConstantForceField(factory);

        registerParameterizedFixedProjectiveConstraint(factory);
        
        registerMeanSquaredErrorMapping(factory);
        registerMeanSquaredErrorMultiMapping(factory);
        registerGeodesicPoseLossMapping(factory);

        registerLoopsControlsGUI();
 
    
    }

}