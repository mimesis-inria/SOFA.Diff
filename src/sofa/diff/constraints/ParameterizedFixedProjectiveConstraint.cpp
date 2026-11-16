#include <sofa/diff/constraints/ParameterizedFixedProjectiveConstraint.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/accessor.h>

#include <utility>

namespace sofadiff
{

template<class DataTypes>
ParameterizedFixedProjectiveConstraint<DataTypes>::ParameterizedFixedProjectiveConstraint()
    : Inherit()
    , d_poseOffset(this->initData(&d_poseOffset, sofa::type::vector<SReal>(N, SReal(0)), "poseOffset",
          "Trainable fixed-pose offset. Vec3d: [x y z]. Rigid3d: [tx ty tz rx ry rz]."))
    , d_componentFlags(this->initData(&d_componentFlags, sofa::type::vector<bool>(N, true), "componentFlags",
          "PoseOffset update mask. Rigid3d order: [tx ty tz rx ry rz]."))
{
}

template<class DataTypes>
void ParameterizedFixedProjectiveConstraint<DataTypes>::init()
{
    Inherit::init();

    m_poseOffsetParameter = this->initParameter(d_poseOffset);
    this->checkForNotImplementedParameters(this->getDataFields());

    captureInitialPositions();
    m_storedBlocks.clear();
}

template<class DataTypes>
void ParameterizedFixedProjectiveConstraint<DataTypes>::reinit()
{
    Inherit::reinit();
    m_storedBlocks.clear();
}

template<class DataTypes>
void ParameterizedFixedProjectiveConstraint<DataTypes>::captureInitialPositions()
{
    if (this->mstate.get() == nullptr)
        return;

    m_initialPositions = this->mstate->read(sofa::core::vec_id::read_access::position)->getValue();
}



template<class DataTypes>
sofa::Index ParameterizedFixedProjectiveConstraint<DataTypes>::parameterBaseForStoredBlock(
    sofa::Index storedBlockId,
    sofa::Size parameterSize) const
{
    if (parameterSize >= static_cast<sofa::Size>(N) * m_storedBlocks.size())
        return static_cast<sofa::Index>(N) * storedBlockId;

    return 0;
}

template<class DataTypes>
bool ParameterizedFixedProjectiveConstraint<DataTypes>::isParameterActive(unsigned int component) const
{
    if (component >= N)
        return false;

    const auto& flags = d_componentFlags.getValue();
    return component >= flags.size() || flags[component];
}

template<class DataTypes>
typename ParameterizedFixedProjectiveConstraint<DataTypes>::Mat3
ParameterizedFixedProjectiveConstraint<DataTypes>::identityMatrix()
{
    Mat3 I;
    I.identity();
    return I;
}

template<class DataTypes>
typename ParameterizedFixedProjectiveConstraint<DataTypes>::Quat
ParameterizedFixedProjectiveConstraint<DataTypes>::identityQuaternion()
{
    Quat q;
    q.identity();
    return q;
}

template<class DataTypes>
typename ParameterizedFixedProjectiveConstraint<DataTypes>::Mat3
ParameterizedFixedProjectiveConstraint<DataTypes>::quaternionToMatrix(const Quat& q)
{
    Mat3 R;
    R.identity();

    if constexpr (std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
        q.toMatrix(R);
    else
        SOFA_UNUSED(q);

    return R;
}

template<class DataTypes>
SReal ParameterizedFixedProjectiveConstraint<DataTypes>::parameterValue(
    const sofa::type::vector<SReal>& poseOffset,
    sofa::Index parameterBase,
    unsigned int component) const
{
    if (component >= N || parameterBase + component >= poseOffset.size())
        return SReal(0);

    return isParameterActive(component) ? poseOffset[parameterBase + component] : SReal(0);
}

template<class DataTypes>
typename ParameterizedFixedProjectiveConstraint<DataTypes>::Vec3
ParameterizedFixedProjectiveConstraint<DataTypes>::parameterTranslation(
    const sofa::type::vector<SReal>& poseOffset,
    sofa::Index parameterBase) const
{
    Vec3 t;
    t[0] = static_cast<Real>(parameterValue(poseOffset, parameterBase, 0));
    t[1] = static_cast<Real>(parameterValue(poseOffset, parameterBase, 1));
    t[2] = static_cast<Real>(parameterValue(poseOffset, parameterBase, 2));
    return t;
}

template<class DataTypes>
typename ParameterizedFixedProjectiveConstraint<DataTypes>::Quat
ParameterizedFixedProjectiveConstraint<DataTypes>::parameterRotation(
    const sofa::type::vector<SReal>& poseOffset,
    sofa::Index parameterBase) const
{
    if constexpr (std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
    {
        Vec3 angles;
        angles[0] = static_cast<Real>(parameterValue(poseOffset, parameterBase, 3));
        angles[1] = static_cast<Real>(parameterValue(poseOffset, parameterBase, 4));
        angles[2] = static_cast<Real>(parameterValue(poseOffset, parameterBase, 5));

        Quat q = Quat::createQuaterFromEuler(angles);
        q.normalize();
        return q;
    }
    else
    {
        SOFA_UNUSED(poseOffset);
        SOFA_UNUSED(parameterBase);
        return identityQuaternion();
    }
}

template<class DataTypes>
typename ParameterizedFixedProjectiveConstraint<DataTypes>::Vec3
ParameterizedFixedProjectiveConstraint<DataTypes>::matVec(const Mat3& A, const Vec3& x)
{
    Vec3 y;
    y.clear();

    for (unsigned int i = 0; i < 3; ++i)
        for (unsigned int j = 0; j < 3; ++j)
            y[i] += A[i][j] * x[j];

    return y;
}

template<class DataTypes>
typename ParameterizedFixedProjectiveConstraint<DataTypes>::Vec3
ParameterizedFixedProjectiveConstraint<DataTypes>::transposeMatVec(const Mat3& A, const Vec3& x)
{
    Vec3 y;
    y.clear();

    for (unsigned int i = 0; i < 3; ++i)
        for (unsigned int j = 0; j < 3; ++j)
            y[i] += A[j][i] * x[j];

    return y;
}

template<class DataTypes>
typename ParameterizedFixedProjectiveConstraint<DataTypes>::Vec3
ParameterizedFixedProjectiveConstraint<DataTypes>::cross(const Vec3& a, const Vec3& b)
{
    Vec3 c;
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
    return c;
}

template<class DataTypes>
typename ParameterizedFixedProjectiveConstraint<DataTypes>::CommandFrame
ParameterizedFixedProjectiveConstraint<DataTypes>::computeCommandFrame(
    const Coord& initialControlled,
    const sofa::type::vector<SReal>& poseOffset,
    sofa::Index parameterBase) const
{
    CommandFrame frame;
    frame.qBase = identityQuaternion();
    frame.qOffset = identityQuaternion();
    frame.qCommand = identityQuaternion();
    frame.RCommand = identityMatrix();
    frame.tLocal.clear();

    if constexpr (!std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
    {
        SOFA_UNUSED(initialControlled);
        SOFA_UNUSED(poseOffset);
        SOFA_UNUSED(parameterBase);
        return frame;
    }
    else
    {
        // Express the commanded rigid pose in the initial controlled-node frame.
        frame.tLocal = parameterTranslation(poseOffset, parameterBase);
        frame.qOffset = parameterRotation(poseOffset, parameterBase);
        frame.qBase = initialControlled.getOrientation();
        frame.qBase.normalize();

        // The parameter rotation also rotates the local translation direction.
        // centerTarget = centerInitial + R(qBase * qOffset) * tLocal.
        frame.qCommand = frame.qBase * frame.qOffset;
        frame.qCommand.normalize();
        frame.RCommand = quaternionToMatrix(frame.qCommand);
        return frame;
    }
}

template<class DataTypes>
void ParameterizedFixedProjectiveConstraint<DataTypes>::applyOffsetToCoord(
    Coord& target,
    const Coord& initialControlled,
    const sofa::type::vector<SReal>& poseOffset,
    sofa::Index parameterBase) const
{
    target = initialControlled;

    if constexpr (std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
    {
        const CommandFrame frame = computeCommandFrame(
            initialControlled, poseOffset, parameterBase);
        const Vec3 tGlobal = matVec(frame.RCommand, frame.tLocal);

        for (unsigned int i = 0; i < 3; ++i)
            target.getCenter()[i] += tGlobal[i];

        // Preserve the original local/right-multiplied orientation update.
        target.getOrientation() = initialControlled.getOrientation() * frame.qOffset;
        target.getOrientation().normalize();
    }
    else
    {
        for (unsigned int c = 0; c < N; ++c)
            target[c] += static_cast<Real>(parameterValue(poseOffset, parameterBase, c));
    }
}

template<class DataTypes>
typename ParameterizedFixedProjectiveConstraint<DataTypes>::VectorN
ParameterizedFixedProjectiveConstraint<DataTypes>::mapRawGradientToParameterGradient(
    const VectorN& rawGradient,
    const Coord& initialControlled,
    const sofa::type::vector<SReal>& poseOffset,
    sofa::Index parameterBase) const
{
    if constexpr (!std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
    {
        SOFA_UNUSED(initialControlled);
        SOFA_UNUSED(poseOffset);
        SOFA_UNUSED(parameterBase);
        return rawGradient;
    }
    else
    {
        const CommandFrame frame = computeCommandFrame(
            initialControlled, poseOffset, parameterBase);

        VectorN mapped;
        mapped.clear();

        Vec3 rawTranslation;
        rawTranslation[0] = static_cast<Real>(rawGradient[0]);
        rawTranslation[1] = static_cast<Real>(rawGradient[1]);
        rawTranslation[2] = static_cast<Real>(rawGradient[2]);

        Vec3 rawRotation;
        rawRotation[0] = static_cast<Real>(rawGradient[3]);
        rawRotation[1] = static_cast<Real>(rawGradient[4]);
        rawRotation[2] = static_cast<Real>(rawGradient[5]);

        // Pull the Cartesian adjoint back into the local command coordinates.
        const Vec3 mappedTranslation = transposeMatVec(frame.RCommand, rawTranslation);
        Vec3 mappedRotation = transposeMatVec(frame.RCommand, rawRotation);

        // The commanded rotation changes the direction of the local translation.
        const Vec3 coupling = cross(frame.tLocal, mappedTranslation);
        for (unsigned int i = 0; i < 3; ++i)
            mappedRotation[i] += coupling[i];

        for (unsigned int i = 0; i < 3; ++i)
        {
            mapped[i] = static_cast<SReal>(mappedTranslation[i]);
            mapped[i + 3] = static_cast<SReal>(mappedRotation[i]);
        }
        return mapped;
    }
}

template<class DataTypes>
void ParameterizedFixedProjectiveConstraint<DataTypes>::projectPosition(
    const sofa::core::MechanicalParams* mparams,
    DataVecCoord& xData)
{
    SOFA_UNUSED(mparams);

    if (this->mstate.get() == nullptr)
        return;

    if (m_initialPositions.empty())
        captureInitialPositions();

    const auto& poseOffset = d_poseOffset.getValue();
    if (poseOffset.size() < N)
    {
        msg_warning() << "poseOffset must contain at least " << N << " values.";
        return;
    }

    helper::WriteAccessor<DataVecCoord> x = helper::getWriteAccessor(xData);

    auto applyOffset = [&](sofa::Index index, sofa::Index parameterBase)
    {
        if (index >= x.size() || index >= m_initialPositions.size())
        {
            msg_warning() << "Constrained index " << index << " is outside the mechanical state.";
            return;
        }

        if (parameterBase + N > poseOffset.size())
        {
            msg_warning() << "poseOffset does not contain enough values for constrained index " << index << ".";
            return;
        }

        Coord target;
        applyOffsetToCoord(target, m_initialPositions[index], poseOffset, parameterBase);
        x[index] = target;
    };

    if (this->d_fixAll.getValue())
    {
        const bool perNodeParameters = poseOffset.size() >= static_cast<sofa::Size>(N) * x.size();
        for (sofa::Index i = 0; i < x.size(); ++i)
            applyOffset(i, perNodeParameters ? static_cast<sofa::Index>(N) * i : 0);
        return;
    }

    const auto& indices = this->d_indices.getValue();
    const bool perNodeParameters = poseOffset.size() >= static_cast<sofa::Size>(N) * indices.size();
    for (sofa::Index k = 0; k < indices.size(); ++k)
        applyOffset(indices[k], perNodeParameters ? static_cast<sofa::Index>(N) * k : 0);
}

template<class DataTypes>
bool ParameterizedFixedProjectiveConstraint<DataTypes>::captureDenseColumns(
    sofa::core::behavior::ZeroDirichletCondition* matrix,
    sofa::Index constrainedIndex,
    sofa::Size matrixSize,
    DenseColumnArray& columns) const
{
    for (unsigned int c = 0; c < N; ++c)
    {
        const sofa::Index scalarDof = static_cast<sofa::Index>(N) * constrainedIndex + c;
        columns[c].clear();

        const bool ok = matrix->copyColumnBeforeDiscard(scalarDof, columns[c], 0, matrixSize);

        if (!ok)
        {
            msg_warning() << "copyColumnBeforeDiscard() failed for index "
                          << constrainedIndex << ", component " << c
                          << ", scalar DoF " << scalarDof << ".";
            return false;
        }

        if (columns[c].size() < matrixSize)
        {
            msg_warning() << "Stored column for scalar DoF " << scalarDof
                          << " has size " << columns[c].size()
                          << " but expected at least " << matrixSize << ".";
            return false;
        }
    }

    return true;
}

template<class DataTypes>
bool ParameterizedFixedProjectiveConstraint<DataTypes>::isZeroBlock(const MatrixN& block)
{
    for (unsigned int r = 0; r < N; ++r)
        for (unsigned int c = 0; c < N; ++c)
            if (block[r][c] != SReal(0))
                return false;

    return true;
}

template<class DataTypes>
void ParameterizedFixedProjectiveConstraint<DataTypes>::compressDenseColumnsToBlocks(
    const DenseColumnArray& columns,
    sofa::Index constrainedIndex,
    sofa::Size matrixSize,
    StoredConstrainedBlock& outputBlock) const
{
    outputBlock.constrainedIndex = constrainedIndex;
    outputBlock.couplingBlocks.clear();

    const sofa::Size nbNodes = matrixSize / N;
    outputBlock.couplingBlocks.reserve(8);

    for (sofa::Index nodeIndex = 0; nodeIndex < nbNodes; ++nodeIndex)
    {
        if (nodeIndex == constrainedIndex)
            continue;

        MatrixN block;
        block.clear();

        const sofa::Index rowBase = static_cast<sofa::Index>(N) * nodeIndex;

        for (unsigned int rowDof = 0; rowDof < N; ++rowDof)
        {
            const sofa::Index scalarRow = rowBase + rowDof;

            for (unsigned int colDof = 0; colDof < N; ++colDof)
                block[rowDof][colDof] = columns[colDof][scalarRow];
        }

        if (isZeroBlock(block))
            continue;

        StoredCouplingBlock coupling;
        coupling.nodeIndex = nodeIndex;
        coupling.stiffness = block;
        outputBlock.couplingBlocks.push_back(coupling);
    }
}

template<class DataTypes>
void ParameterizedFixedProjectiveConstraint<DataTypes>::applyConstraint(sofa::core::behavior::ZeroDirichletCondition* matrix)
{
    m_storedBlocks.clear();

    if (matrix != nullptr && this->mstate.get() != nullptr)
    {
        const sofa::Size matrixSize = this->mstate->getMatrixSize();

        auto storeIndex = [&](sofa::Index index)
        {
            DenseColumnArray denseColumns;

            if (!captureDenseColumns(matrix, index, matrixSize, denseColumns))
                return;

            StoredConstrainedBlock block;
            compressDenseColumnsToBlocks(denseColumns, index, matrixSize, block);

            msg_info() << "Stored " << block.couplingBlocks.size()
                       << " nonzero coupling blocks for constrained index " << index << ".";

            m_storedBlocks.push_back(std::move(block));
        };

        if (this->d_fixAll.getValue())
        {
            const sofa::Size nbDofs = matrixSize / N;
            m_storedBlocks.reserve(nbDofs);

            for (sofa::Index index = 0; index < nbDofs; ++index)
                storeIndex(index);
        }
        else
        {
            const auto& indices = this->d_indices.getValue();
            m_storedBlocks.reserve(indices.size());

            for (const sofa::Index index : indices)
                storeIndex(index);
        }
    }

    Inherit::applyConstraint(matrix);
}

template<class DataTypes>
void ParameterizedFixedProjectiveConstraint<DataTypes>::applyParametersJacobianTranspose(
    const sofa::core::MechanicalParams* mparams,
    sofa::core::MultiVecDerivId vecId)
{
    SOFA_UNUSED(mparams);

    if (m_poseOffsetParameter == nullptr || this->mstate.get() == nullptr)
        return;

    const auto state = this->mstate.get();
    const auto& vectorData = *vecId[state].read();
    const helper::ReadAccessor<sofa::Data<VecDeriv>> adjoint = helper::getReadAccessor(vectorData);
    const auto& poseOffset = d_poseOffset.getValue();

    auto poseOffsetGradient = helper::getWriteAccessor(m_poseOffsetParameter->d_gradient);
    const sofa::Size parameterSize = poseOffset.size();
    if (poseOffsetGradient.size() < parameterSize)
        poseOffsetGradient.resize(parameterSize);

    for (sofa::Index blockId = 0; blockId < m_storedBlocks.size(); ++blockId)
    {
        const StoredConstrainedBlock& constrainedBlock = m_storedBlocks[blockId];
        const sofa::Index poseOffsetBase = parameterBaseForStoredBlock(blockId, poseOffsetGradient.size());

        if (poseOffsetBase + N > poseOffsetGradient.size() || poseOffsetBase + N > poseOffset.size())
            continue;

        VectorN rawGradient;
        rawGradient.clear();
        for (const StoredCouplingBlock& couplingBlock : constrainedBlock.couplingBlocks)
        {
            if (couplingBlock.nodeIndex >= adjoint.size())
                continue;

            const Deriv& lambda = adjoint[couplingBlock.nodeIndex];
            for (unsigned int colDof = 0; colDof < N; ++colDof)
            {
                SReal dot = SReal(0);
                for (unsigned int rowDof = 0; rowDof < N; ++rowDof)
                    dot += static_cast<SReal>(lambda[rowDof]) * SReal(-1) * couplingBlock.stiffness[rowDof][colDof];
                rawGradient[colDof] += dot;
            }
        }

        const VectorN mappedGradient = mapRawGradientToParameterGradient(
            rawGradient,
            m_initialPositions[constrainedBlock.constrainedIndex],
            poseOffset,
            poseOffsetBase);

        for (unsigned int k = 0; k < N; ++k)
        {
            if (!isParameterActive(k))
                continue;
            poseOffsetGradient[poseOffsetBase + k] += mappedGradient[k];
        }
    }
}

void registerParameterizedFixedProjectiveConstraint(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData(
            "Parameterized FixedProjectiveConstraint with trainable initial-frame poseOffset for Vec3d and Rigid3d.")
        .add<ParameterizedFixedProjectiveConstraint<sofa::defaulttype::Vec3Types>>()
        .add<ParameterizedFixedProjectiveConstraint<sofa::defaulttype::Rigid3Types>>()
    );
}

template class ParameterizedFixedProjectiveConstraint<sofa::defaulttype::Vec3Types>;
template class ParameterizedFixedProjectiveConstraint<sofa::defaulttype::Rigid3Types>;

} // namespace sofadiff
