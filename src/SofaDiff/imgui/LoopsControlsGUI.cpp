/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program. If not, see <http://www.gnu.org/licenses/>.              *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <SofaDiff/imgui/LoopsControlsGUI.h>
#include <SofaDiff/DifferentiableAnimationLoop.h>
#include <SofaDiff/optimizers/OptimizationLoop.h>

#include <imgui.h>
#include <SofaImGui/guis/AdditionalGUIRegistry.h>
#include <SofaImGui/ImGuiDataWidget.h>
#include <IconsFontAwesome6.h>

#include <sofa/component/visual/BaseCamera.h>
#include <sofa/simulation/Simulation.h>

namespace sofadiff
{

void registerLoopsControlsGUI()
{
    sofaimgui::guis::MainAdditionGUIRegistry::registerAdditionalGUI(new LoopsControlsGUI());
}

void LoopsControlsGUI::doDraw(core::sptr<simulation::Node> groot)
{
    OptimizationLoop * optimizationLoop;
    groot->get(optimizationLoop);
    if (optimizationLoop)
    {
        const auto params = core::execparams::defaultInstance();
        const auto dt = groot->getDt();

        ImGui::Text("Optimization Loop");
        ImGui::SameLine();
        const auto forwardStepButton = ImGui::Button(std::string(ICON_FA_FORWARD_STEP).append("##1").c_str());
        ImGui::SetItemTooltip("Make one step of the forward solver");
        if (forwardStepButton)
        {
            optimizationLoop->step(params, dt);
            simulation::node::updateVisual(groot.get());
        }
    }

    DifferentiableAnimationLoop * differentiableLoop;
    groot->get(differentiableLoop);
    if (differentiableLoop)
    {
        const auto params = core::execparams::defaultInstance();
        const auto dt = groot->getDt();

        ImGui::Text("Differentiable Animation Loop");
        ImGui::SameLine();
        const auto adjointStepButton = ImGui::Button(ICON_FA_BACKWARD_STEP);
        ImGui::SetItemTooltip("Make one step of the adjoint solver");
        ImGui::SameLine();
        const auto forwardStepButton = ImGui::Button(ICON_FA_FORWARD_STEP);
        ImGui::SetItemTooltip("Make one step of the forward solver");
        if(adjointStepButton)
        {
            differentiableLoop->stepAdjoint(params, dt);
            simulation::node::updateVisual(groot.get());
        }
        if (forwardStepButton)
        {
            differentiableLoop->step(params, dt);
            simulation::node::updateVisual(groot.get());
        }
    }
    else
    {
        ImGui::Text("No differentiable animation loop");
    }
}


std::string LoopsControlsGUI::getWindowName() const
{
    return "Loops Controls";
}


std::string LoopsControlsGUI::getWindowIcon() const
{
    return ICON_FA_BRAIN;
}

}
