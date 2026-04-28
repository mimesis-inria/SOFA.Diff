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

#include <SofaDiff/config.h>
#include <SofaDiff/optimizers/GradientBasedOptimizationLoop.h>

#include <SofaPython3/PythonFactory.h>
#include <SofaPython3/Sofa/Core/Binding_Base.h>
#include <sofa/core/ExecParams.h>


namespace sofapython3
{
namespace py { using namespace pybind11; }

class GradientBasedOptimizationLoop_Trampoline: public sofadiff::GradientBasedOptimizationLoop
{
public:
    SOFA_CLASS(GradientBasedOptimizationLoop_Trampoline, sofadiff::GradientBasedOptimizationLoop);

    using GradientBasedOptimizationLoop::GradientBasedOptimizationLoop;  /* Inherit constructors */
    static py_shared_ptr<GradientBasedOptimizationLoop_Trampoline> create(const py::args& args, const py::kwargs& kwargs);

    void _allocate() override;
    void _initialize() override;
    void _updateParameters() override;

    std::string getClassName() const override;
};

void moduleAddGradientBasedOptimizationLoop(const pybind11::module& m);

}
