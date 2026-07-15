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
#include <sofa/diff/bindings/Binding_GradientBasedOptimizationLoop.h>
#include <sofa/diff/bindings/Binding_OptimizationLoop.h>
#include <sofa/diff/bindings/Binding_Parameter.h>
#include <sofa/diff/bindings/Binding_ParameterizedForceField.h>

#include <pybind11/pybind11.h>


namespace py
{
using namespace pybind11;
}

namespace sofapython3
{

PYBIND11_MODULE(sofadiff, m)
{
    py::module_::import("Sofa");
    moduleAddParameter(m);
    moduleAddOptimizationLoop(m);
    moduleAddGradientBasedOptimizationLoop(m);
    moduleAddParameterizedForceField(m);
}

}