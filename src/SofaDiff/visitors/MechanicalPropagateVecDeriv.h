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

#include <sofa/simulation/MechanicalVisitor.h>


namespace sofadiff
{

class SOFA_SOFADIFF_API MechanicalPropagateVecDeriv : public simulation::MechanicalVisitor
{
public:
    MechanicalPropagateVecDeriv(const core::MechanicalParams* params, const core::MultiVecDerivId& vecId);

    Result fwdMechanicalMapping(VisitorContext* ctx, core::BaseMapping* map) override;
    bool stopAtMechanicalMapping(simulation::Node*, core::BaseMapping*) override;

    bool isThreadSafe() const override;

    const char* getClassName() const override;
    std::string getInfos() const override;

private:
    core::MultiVecDerivId m_vecId;
};


}
