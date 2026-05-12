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
#pragma once

#include <sofa/diff/config.h>
#include <sofa/diff/DifferentiableAnimationLoop.h>
#include <sofa/diff/optimizers/OptimizationLoop.h>

#include <SofaImGui/guis/BaseAdditionalGUI.h>



using sofaimgui::guis::BaseAdditionalGUI;

namespace sofadiff
{

/**
 * @brief Example custom GUI module for testing the injection system.
 */
class SOFA_SOFADIFF_API LoopsControlsGUI : public BaseAdditionalGUI
{
public:
    LoopsControlsGUI(): m_recording(false) {}

    std::string getWindowName() const override;
    std::string getWindowIcon() const override;
private:
    bool m_recording;

    void doDraw(core::sptr<simulation::Node> groot) override;

    static void doDrawOptimizer(const core::sptr<simulation::Node>& groot, OptimizationLoop * optimizer);
    static void doDrawDifferentiableSimulator(const core::sptr<simulation::Node>& groot, DifferentiableAnimationLoop * simulator);
    static void doDrawDefaultSimulator(const core::sptr<simulation::Node>& groot, simulation::DefaultAnimationLoop * simulator);
};

}
