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
#include <SofaDiff/DistanceToPointCost.inl>

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/component/mapping/linear/config.h>

namespace sofadiff
{

using namespace sofa::defaulttype;

void registerDistanceToPointCost(core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("Cost function that computes the distance to a point")
        .add< DistanceToPointCost<Vec3Types> >()
        .add< DistanceToPointCost<Vec2Types> >()
        .add< DistanceToPointCost<Vec6Types> >()
        .add< DistanceToPointCost<Rigid3Types> >()
        // .add< DistanceToPointCost<Rigid2Types> >()
    );
}

template class SOFA_SOFADIFF_API DistanceToPointCost<Vec3Types> ;
template class SOFA_SOFADIFF_API DistanceToPointCost<Vec2Types> ;
template class SOFA_SOFADIFF_API DistanceToPointCost<Vec6Types> ;
template class SOFA_SOFADIFF_API DistanceToPointCost<Rigid3Types>;
// template class SOFA_SOFADIFF_API DistanceToPointCost<Rigid2Types>;

}
