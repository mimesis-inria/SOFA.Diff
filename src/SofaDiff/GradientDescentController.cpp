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

#include <SofaDiff/GradientDescentController.h>
#include <SofaDiff/ComputeCostGradientVisitor.h>

#include <sofa/core/ObjectFactory.h>


namespace sofadiff {

GradientDescentController::GradientDescentController()
{
  this->f_listening.setValue(true);
}

void GradientDescentController::init()
{
}

void GradientDescentController::onEndAnimationStep(const double /*dt*/)
{
  std::cout << "SofaDiff::GradientDescentController::onEndAnimationStep" << std::endl;

  auto* ctx = this->getContext();
  auto* params = core::mechanicalparams::defaultInstance();
  ComputeCostGradientVisitor(params).execute(ctx, false);
}

void registerGradientDescentController(core::ObjectFactory* factory)
{
  factory->registerObjects(core::ObjectRegistrationData("Controller that performs gradient descent.").add< GradientDescentController >());
}

}