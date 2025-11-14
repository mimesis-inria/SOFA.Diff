/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
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
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <SofaDiff/config.h>

#include <sofa/core/ObjectFactory.h>
using sofa::core::ObjectFactory;
#include <sofa/helper/system/PluginManager.h>

namespace sofadiff
{
    extern void registerGradientDescentController(sofa::core::ObjectFactory* factory);
    extern void registerParameterizedSpringForceField(sofa::core::ObjectFactory* factory);
    extern void registerTrainableParameter(sofa::core::ObjectFactory* factory);
    extern void registerLossState(sofa::core::ObjectFactory* factory);
    extern void registerParameterizedTetrahedronFEMForceField(sofa::core::ObjectFactory* factory);

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
        static bool first = true;
        if (first)
        {
            first = false;

            // make sure that this plugin is registered into the PluginManager
            sofa::helper::system::PluginManager::getInstance().registerPlugin(MODULE_NAME);
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
        registerGradientDescentController(factory);
        registerParameterizedSpringForceField(factory);
        registerParameterizedTetrahedronFEMForceField(factory);
        registerTrainableParameter(factory);
        registerLossState(factory);
    }

}