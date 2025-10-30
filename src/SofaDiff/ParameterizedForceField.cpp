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

#include <SofaDiff/ParameterizedForceField.h>
#include <SofaDiff/TrainableParameter.h>


namespace sofadiff
{

const std::vector<double> & ParameterizedForceField::getParameterGradient(const std::string & dataName)
{
    // No need to check for existence of key because we check that the data exists in GradientDescentController
    return m_gradientMap[dataName];
}

void ParameterizedForceField::addToDataGradient(const Data<type::vector<SReal> > &data, const std::vector<double> &gradient)
{
    const auto * dataPtr = data.getParent();
    if (dataPtr == nullptr)
        return;

    auto * parameterPtr = dynamic_cast<TrainableParameterVector *>(dataPtr->getOwner());
    if (parameterPtr == nullptr)
        return;

    // TODO: optimize things below?
    auto dataGradientAccessor = helper::getWriteAccessor(parameterPtr->d_gradient);
    if (dataGradientAccessor.size() != gradient.size())
    {
        // TODO: use msg_error()? (unaccessible here, maybe because we are not a BaseObject?)
        std::cout << "Size mismatch in gradient for TrainableParameter from ParameterizedForceField: " << dataGradientAccessor.size() << " != " << gradient.size() << std::endl;
        return;
    }
    for (unsigned int i = 0; i < gradient.size(); ++i)
        dataGradientAccessor[i] += gradient[i];
}


}
