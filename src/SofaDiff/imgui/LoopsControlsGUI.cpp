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
using namespace sofa::core::objectmodel;

void registerLoopsControlsGUI()
{
    sofaimgui::guis::MainAdditionGUIRegistry::registerAdditionalGUI(new LoopsControlsGUI());
}

void LoopsControlsGUI::doDraw(core::sptr<simulation::Node> groot)
{
    std::vector<core::behavior::BaseAnimationLoop*> loops;
    groot->get<core::behavior::BaseAnimationLoop>(&loops, BaseContext::SearchDown);
    for (auto * loop : loops)
    {
        if (auto * optimizer = dynamic_cast<OptimizationLoop*>(loop))
            doDrawOptimizer(groot, optimizer);

        else if (auto * differentiableSimulator = dynamic_cast<DifferentiableAnimationLoop*>(loop))
            doDrawDifferentiableSimulator(groot, differentiableSimulator);

        else if (auto * defaultSimulator = dynamic_cast<simulation::DefaultAnimationLoop*>(loop))
            doDrawDefaultSimulator(groot, defaultSimulator);
    }
}

void LoopsControlsGUI::doDrawOptimizer(const core::sptr<simulation::Node>& groot, OptimizationLoop * optimizer)
{
    const auto params = core::execparams::defaultInstance();
    const auto dt = groot->getDt();
    bool clicked = false;
    std::string suffix = std::string("##").append(optimizer->getName());

    ImGui::Text("Optimization Loop");

    ImGui::SameLine(150);
    ImGui::BeginDisabled(!optimizer->isStepAllowed());
    ImGui::PushButtonRepeat(true);
    clicked = ImGui::Button(std::string(ICON_FA_FORWARD_STEP).append(suffix).c_str());
    ImGui::SetItemTooltip("Make one iteration of the optimizer");
    ImGui::PopButtonRepeat();
    ImGui::EndDisabled();
    if (clicked)
        optimizer->step(params, dt);

    ImGui::SameLine();
    ImGui::BeginDisabled(!optimizer->isStepAllowed() || optimizer->getMaxOptimizationSteps() <= 0);
    clicked = ImGui::Button(std::string(ICON_FA_FORWARD_FAST).append(suffix).c_str());
    ImGui::SetItemTooltip("Make all the iterations of the optimizer");
    ImGui::EndDisabled();
    if (clicked)
        while (optimizer->isStepAllowed())
            optimizer->step(params, dt);

    ImGui::SameLine();
    ImGui::Text("%d", optimizer->getCurrentOptimizationStep());
    ImGui::SetItemTooltip("Current number of optimization steps performed");
    ImGui::SameLine(260);
    ImGui::Text("/");

    int maxOptimizationSteps = optimizer->getMaxOptimizationSteps();
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120);
    clicked = ImGui::InputInt(suffix.append("-maxOptimizationSteps").c_str(), &maxOptimizationSteps);
    ImGui::SetItemTooltip("Maximum number of optimization steps to perform");
    if (clicked)
        optimizer->setMaxOptimizationSteps(maxOptimizationSteps);

    int maxSimulationSteps = optimizer->getMaxSimulationSteps();
    ImGui::SameLine(0, 30);
    ImGui::SetNextItemWidth(120);
    clicked = ImGui::InputInt(suffix.append("-maxSimulationSteps").c_str(), &maxSimulationSteps);
    ImGui::SetItemTooltip("Maximum number of simulation steps to optimize");
    if (clicked)
        optimizer->setMaxSimulationSteps(maxSimulationSteps);
}

void LoopsControlsGUI::doDrawDifferentiableSimulator(const core::sptr<simulation::Node>& groot, DifferentiableAnimationLoop * simulator)
{
    const auto params = core::execparams::defaultInstance();
    const auto dt = groot->getDt();
    bool clicked = false;
    std::string suffix = std::string("##").append(simulator->getName());

    ImGui::Text("Simulation Loop");

    ImGui::SameLine(150);
    ImGui::BeginDisabled(!simulator->isStepAllowed());
    ImGui::PushButtonRepeat(true);
    clicked = ImGui::Button(std::string(ICON_FA_FORWARD_STEP).append(suffix).c_str());
    ImGui::SetItemTooltip("Make one step of the forward solver");
    ImGui::PopButtonRepeat();
    ImGui::EndDisabled();
    if (clicked)
        simulator->step(params, dt);

    ImGui::SameLine();
    ImGui::BeginDisabled(!simulator->isStepAllowed() || simulator->getMaxSimulationSteps() <= 0);
    clicked = ImGui::Button(std::string(ICON_FA_FORWARD_FAST).append(suffix).c_str());
    ImGui::SetItemTooltip("Make all the steps of the forward solver");
    ImGui::EndDisabled();
    if (clicked)
        while (simulator->isStepAllowed())
            simulator->step(params, dt);

    ImGui::SameLine();
    ImGui::Text("%d", simulator->getCurrentSimulationStep());
    ImGui::SetItemTooltip("Current number of simulation steps performed");
    ImGui::SameLine(260);
    ImGui::Text("/");

    int totalTimesteps = simulator->getMaxSimulationSteps();
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120);
    clicked = ImGui::InputInt(suffix.append("-totalTimesteps").c_str(), &totalTimesteps);
    ImGui::SetItemTooltip("Number of simulation steps to perform for the simulation to be complete.");
    if (clicked)
        simulator->setMaxSimulationSteps(totalTimesteps);

    ImGui::SameLine(0, 30);
    ImGui::BeginDisabled(!simulator->isStepAdjointAllowed());
    ImGui::PushButtonRepeat(true);
    clicked = ImGui::Button(std::string(ICON_FA_BACKWARD_STEP).append(suffix).c_str());
    ImGui::SetItemTooltip("Make one step of the adjoint solver");
    ImGui::PopButtonRepeat();
    ImGui::EndDisabled();
    if (clicked)
        simulator->stepAdjoint(params, dt);

    ImGui::SameLine();
    ImGui::BeginDisabled(true);
    clicked = ImGui::Button(std::string(ICON_FA_BACKWARD_FAST).append(suffix).c_str());
    ImGui::SetItemTooltip("Make all the steps of the adjoint solver");
    ImGui::EndDisabled();
    if (clicked)
        while (simulator->isStepAdjointAllowed())
            simulator->stepAdjoint(params, dt);
}

void LoopsControlsGUI::doDrawDefaultSimulator(const core::sptr<simulation::Node> &groot, simulation::DefaultAnimationLoop *simulator)
{
    const auto params = core::execparams::defaultInstance();
    const auto dt = groot->getDt();
    bool clicked = false;
    std::string suffix = std::string("##").append(simulator->getName());

    ImGui::Text("Simulation Loop");

    ImGui::SameLine(150);
    ImGui::PushButtonRepeat(true);
    clicked = ImGui::Button(std::string(ICON_FA_FORWARD_STEP).append(suffix).c_str());
    ImGui::SetItemTooltip("Make one step of the forward solver");
    ImGui::PopButtonRepeat();
    if (clicked)
        simulator->step(params, dt);
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
