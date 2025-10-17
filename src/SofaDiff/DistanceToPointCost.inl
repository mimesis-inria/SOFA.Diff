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
#pragma once
#include <SofaDiff/DistanceToPointCost.h>

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/linearalgebra/EigenSparseMatrix.h>
#include <sofa/type/Vec.h>


namespace sofadiff
{

template <class TIn>
DistanceToPointCost<TIn>::DistanceToPointCost()
: Inherit()
, d_planeNormal(initData(&d_planeNormal,"planeNormal","Normal of the plane to compute the distance to"))
, d_planePoint(initData(&d_planePoint,"planePoint","A point belonging to the plane"))
{

}


template <class TIn>
void DistanceToPointCost<TIn>::init()
{
    Inherit::init();

    //Normalize plane normal
    const SReal normalNorm = d_planeNormal.getValue().norm();
    if (normalNorm<std::numeric_limits<SReal>::epsilon())
    {
        msg_error()<<" planeNormal data has null norm.";
        this->d_componentState.setValue(core::objectmodel::ComponentState::Invalid);
        return;
    }
    d_planeNormal.setValue(d_planeNormal.getValue()/normalNorm);

    constexpr Size inDerivSize = Deriv_t<TIn>::size();
    constexpr Size inSpatialDimension = Deriv_t<TIn>::spatial_dimensions;
    Size inSize = this->getFromModel()->getSize();
    this->getToModel()->resize( inSize );

    const auto planeNormal = d_planeNormal.getValue();

    J.compressedMatrix.resize( inSize, inSize*inDerivSize );

    for (Size i = 0; i < inSize; i++)
    {
        const size_t col = i * inDerivSize;
        J.compressedMatrix.startVec(i);

        for (Size j = 0; j < inSpatialDimension; j++ )
        {
            J.compressedMatrix.insertBack( i, col + j ) = planeNormal[j];
        }

    }
    J.compressedMatrix.finalize();

    this->d_componentState.setValue(core::objectmodel::ComponentState::Valid);
}

template <class TIn>
void DistanceToPointCost<TIn>::apply(const core::MechanicalParams *mparams, Data<VecCoord_t<TOut>>& out, const Data<VecCoord_t<TIn>>& in)
{
    SOFA_UNUSED(mparams);

    if (this-> d_componentState.getValue() != core::objectmodel::ComponentState::Valid)
        return;

    auto writeOut = helper::getWriteAccessor(out);
    const auto readIn = helper::getReadAccessor(in);

    for (unsigned i=0; i<readIn.size(); i++)
    {
        m_difference = TIn::getCPos(readIn[i]) - d_planePoint.getValue();
        writeOut[i] = type::dot(m_difference, m_difference);
    }
}

template <class TIn>
void DistanceToPointCost<TIn>::applyJ(const core::MechanicalParams *mparams, Data<VecDeriv_t<TOut>>& out, const Data<VecDeriv_t<TIn>>& in)
{
    SOFA_UNUSED(mparams);

    if (this-> d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    auto writeOut = helper::getWriteAccessor(out);
    const auto readIn = helper::getReadAccessor(in);

    for ( unsigned i = 0; i<readIn.size(); i++ )
    {
        writeOut[i] = 2 * type::dot(m_difference, TIn::getDPos(readIn[i]));
    }
}

template <class TIn>
void DistanceToPointCost<TIn>::applyJT(const core::MechanicalParams *mparams, Data<VecDeriv_t<TIn>>& out, const Data<VecDeriv_t<TOut>>& in)
{
    SOFA_UNUSED(mparams);

    if (this-> d_componentState.getValue() != core::objectmodel::ComponentState::Valid)
        return;

    this->resetOutputGradient();

    auto writeOut = helper::getWriteAccessor(out);
    const auto readIn = helper::getReadAccessor(in);
    for ( unsigned i = 0; i<readIn.size(); i++ )
    {
        TIn::setDPos(writeOut[i], TIn::getDPos(writeOut[i]) + 2.0 * m_difference * readIn[i][0]);
    }
}

template <class TIn>
void DistanceToPointCost<TIn>::applyJT(const core::ConstraintParams *cparams, Data<MatrixDeriv_t<TIn>>& out, const Data<MatrixDeriv_t<TOut>>& in)
{
    SOFA_UNUSED(cparams);

    if (this-> d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    auto writeMatrixOut = helper::getWriteAccessor(out);
    const auto readMatrixIn = helper::getReadAccessor(in);

    const auto planeNormal = d_planeNormal.getValue();

    for (auto rowIt = readMatrixIn->begin(); rowIt != readMatrixIn->end(); ++rowIt)
    {
        auto colIt = rowIt.begin();
        auto colItEnd = rowIt.end();
        // Creates a constraints if the input constraint is not empty.
        if (colIt != colItEnd)
        {
            auto o = writeMatrixOut->writeLine(rowIt.index());
            while (colIt != colItEnd)
            {
                Deriv_t<TIn> normal;
                TIn::setDPos(normal,planeNormal*colIt.val()[0]);
                o.addCol(colIt.index(), normal);

                ++colIt;
            }
        }
    }
}

template <class TIn>
const linearalgebra::BaseMatrix* DistanceToPointCost<TIn>::getJ()
{
    return &J;
}

template <class TIn>
void DistanceToPointCost<TIn>::handleTopologyChange()
{
    if (this-> d_componentState.getValue() != sofa::core::objectmodel::ComponentState::Valid)
        return;

    if ( this->toModel && this->fromModel && this->toModel->getSize() != this->fromModel->getSize())
    {
        this->init();
    }
}

};

