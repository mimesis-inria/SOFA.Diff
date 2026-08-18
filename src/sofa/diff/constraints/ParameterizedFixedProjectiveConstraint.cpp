#include <sofa/diff/constraints/ParameterizedFixedProjectiveConstraint.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/accessor.h>

#include <algorithm>
#include <cmath>
#include <cctype>
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
    , d_frameMode(this->initData(&d_frameMode, std::string("initial"), "frameMode",
          "Rigid3d local command frame. Supported values: initial, tangent."))
    , d_tangentStartIndex(this->initData(&d_tangentStartIndex, 1, "tangentStartIndex",
          "Start node used to compute the geometric tangent frame in frameMode='tangent'."))
    , d_tangentEndIndex(this->initData(&d_tangentEndIndex, 20, "tangentEndIndex",
          "End node used to compute the geometric tangent frame in frameMode='tangent'."))
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
    invalidateTangentCache();
}

template<class DataTypes>
void ParameterizedFixedProjectiveConstraint<DataTypes>::reinit()
{
    Inherit::reinit();
    m_storedBlocks.clear();

    // Reinit is used as the optimization-step boundary. Do NOT recapture initial positions here.
    // Promote the tangent frame observed during the previous forward/adjoint solve so the next
    // forward solve uses a frozen, already-deformed tangent frame.
    if (!promotePendingTangentCache())
    {
        // Bootstrap/fallback: if no pending observation exists, try to cache from the current
        // mechanical state. In the optimization loop this is usually the final state from the
        // previous displayed/solved forward pass, before computeLoss() restores the starting state.
        refreshActiveTangentCacheFromCurrentState("reinit fallback");
    }
}

template<class DataTypes>
void ParameterizedFixedProjectiveConstraint<DataTypes>::captureInitialPositions()
{
    if (this->mstate.get() == nullptr)
        return;

    m_initialPositions = this->mstate->read(sofa::core::vec_id::read_access::position)->getValue();
    invalidateTangentCache();
}

template<class DataTypes>
typename ParameterizedFixedProjectiveConstraint<DataTypes>::FrameMode
ParameterizedFixedProjectiveConstraint<DataTypes>::frameMode() const
{
    std::string key = d_frameMode.getValue();
    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (key == "initial" || key == "local" || key == "reference" || key == "ref")
        return FrameMode::Initial;

    if (key == "tangent" || key == "geom" || key == "geometric" || key == "guide")
        return FrameMode::Tangent;

    msg_warning() << "Unsupported frameMode='" << d_frameMode.getValue()
                  << "'. Supported values are 'initial' and 'tangent'. Falling back to 'initial'.";
    return FrameMode::Initial;
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
void ParameterizedFixedProjectiveConstraint<DataTypes>::invalidateTangentCache()
{
    m_cachedTangentDelta = identityQuaternion();
    m_cachedTangentDeltaValid = false;
    m_cachedConstrainedIndex = static_cast<sofa::Index>(-1);
    m_cachedTangentStartIndex = static_cast<sofa::Index>(-1);
    m_cachedTangentEndIndex = static_cast<sofa::Index>(-1);

    m_pendingTangentDelta = identityQuaternion();
    m_pendingTangentDeltaValid = false;
    m_pendingConstrainedIndex = static_cast<sofa::Index>(-1);
    m_pendingTangentStartIndex = static_cast<sofa::Index>(-1);
    m_pendingTangentEndIndex = static_cast<sofa::Index>(-1);
}

template<class DataTypes>
bool ParameterizedFixedProjectiveConstraint<DataTypes>::promotePendingTangentCache()
{
    if constexpr (!std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
        return false;
    else
    {
        if (frameMode() != FrameMode::Tangent)
        {
            m_cachedTangentDelta = identityQuaternion();
            m_cachedTangentDeltaValid = false;
            return false;
        }

        if (!m_pendingTangentDeltaValid)
            return false;

        m_cachedTangentDelta = m_pendingTangentDelta;
        m_cachedTangentDelta.normalize();
        m_cachedTangentDeltaValid = true;
        m_cachedConstrainedIndex = m_pendingConstrainedIndex;
        m_cachedTangentStartIndex = m_pendingTangentStartIndex;
        m_cachedTangentEndIndex = m_pendingTangentEndIndex;

        msg_info() << "Promoted pending geometric tangent frame from nodes "
                   << m_cachedTangentStartIndex << " -> " << m_cachedTangentEndIndex
                   << " for constrained index " << m_cachedConstrainedIndex
                   << " qDelta=[" << m_cachedTangentDelta[0] << ", " << m_cachedTangentDelta[1]
                   << ", " << m_cachedTangentDelta[2] << ", " << m_cachedTangentDelta[3]
                   << "].";

        m_pendingTangentDelta = identityQuaternion();
        m_pendingTangentDeltaValid = false;
        m_pendingConstrainedIndex = static_cast<sofa::Index>(-1);
        m_pendingTangentStartIndex = static_cast<sofa::Index>(-1);
        m_pendingTangentEndIndex = static_cast<sofa::Index>(-1);
        return true;
    }
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
typename ParameterizedFixedProjectiveConstraint<DataTypes>::Quat
ParameterizedFixedProjectiveConstraint<DataTypes>::normalizedQuaternion(Quat q)
{
    if constexpr (std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
        q.normalize();

    return q;
}

template<class DataTypes>
typename ParameterizedFixedProjectiveConstraint<DataTypes>::Quat
ParameterizedFixedProjectiveConstraint<DataTypes>::inverseQuaternion(Quat q)
{
    if constexpr (std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
    {
        q.normalize();
        return q.inverse();
    }
    else
    {
        SOFA_UNUSED(q);
        return identityQuaternion();
    }
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
typename ParameterizedFixedProjectiveConstraint<DataTypes>::Real
ParameterizedFixedProjectiveConstraint<DataTypes>::dot(const Vec3& a, const Vec3& b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

template<class DataTypes>
typename ParameterizedFixedProjectiveConstraint<DataTypes>::Real
ParameterizedFixedProjectiveConstraint<DataTypes>::norm(const Vec3& v)
{
    return static_cast<Real>(std::sqrt(static_cast<double>(dot(v, v))));
}

template<class DataTypes>
typename ParameterizedFixedProjectiveConstraint<DataTypes>::Vec3
ParameterizedFixedProjectiveConstraint<DataTypes>::normalizedVector(const Vec3& v)
{
    const Real n = norm(v);
    Vec3 out;
    out.clear();

    if (n <= Real(1e-12))
        return out;

    for (unsigned int i = 0; i < 3; ++i)
        out[i] = v[i] / n;

    return out;
}

template<class DataTypes>
typename ParameterizedFixedProjectiveConstraint<DataTypes>::Quat
ParameterizedFixedProjectiveConstraint<DataTypes>::shortestArcQuaternion(const Vec3& from, const Vec3& to)
{
    Vec3 a = normalizedVector(from);
    Vec3 b = normalizedVector(to);

    if (norm(a) <= Real(1e-12) || norm(b) <= Real(1e-12))
        return identityQuaternion();

    Real c = dot(a, b);
    c = std::max(Real(-1), std::min(Real(1), c));

    if (c > Real(1) - Real(1e-10))
        return identityQuaternion();

    if (c < Real(-1) + Real(1e-10))
    {
        Vec3 axis;
        axis.clear();

        if (std::abs(a[0]) < Real(0.9))
        {
            Vec3 ex;
            ex[0] = Real(1); ex[1] = Real(0); ex[2] = Real(0);
            axis = normalizedVector(cross(a, ex));
        }
        else
        {
            Vec3 ey;
            ey[0] = Real(0); ey[1] = Real(1); ey[2] = Real(0);
            axis = normalizedVector(cross(a, ey));
        }

        Quat q(axis[0], axis[1], axis[2], Real(0));
        q.normalize();
        return q;
    }

    const Vec3 v = cross(a, b);
    const Real s = static_cast<Real>(std::sqrt(static_cast<double>((Real(1) + c) * Real(2))));
    const Real invS = Real(1) / s;

    Quat q(v[0] * invS, v[1] * invS, v[2] * invS, s * Real(0.5));
    q.normalize();
    return q;
}

template<class DataTypes>
bool ParameterizedFixedProjectiveConstraint<DataTypes>::computeTangentDeltaFromState(
    const VecCoord& currentPositions,
    sofa::Index constrainedIndex,
    Quat& tangentDelta,
    sofa::Index& usedStartIndex,
    sofa::Index& usedEndIndex) const
{
    tangentDelta = identityQuaternion();
    usedStartIndex = static_cast<sofa::Index>(-1);
    usedEndIndex = static_cast<sofa::Index>(-1);

    if constexpr (!std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
    {
        SOFA_UNUSED(currentPositions);
        SOFA_UNUSED(constrainedIndex);
        return false;
    }
    else
    {
        if (frameMode() != FrameMode::Tangent)
            return false;

        if (currentPositions.empty() || m_initialPositions.empty())
        {
            msg_warning() << "Tangent frame requested before positions were available. Falling back to initial frame.";
            return false;
        }

        const int startInt = d_tangentStartIndex.getValue();
        const int endInt = d_tangentEndIndex.getValue();

        if (startInt < 0 || endInt < 0)
        {
            msg_warning() << "Invalid tangent window [" << startInt << ", " << endInt
                          << "]. Falling back to initial frame.";
            return false;
        }

        const sofa::Index startIndex = static_cast<sofa::Index>(startInt);
        const sofa::Index endIndex = static_cast<sofa::Index>(endInt);

        if (startIndex >= currentPositions.size() || endIndex >= currentPositions.size() ||
            startIndex >= m_initialPositions.size() || endIndex >= m_initialPositions.size())
        {
            msg_warning() << "Tangent window [" << startIndex << ", " << endIndex
                          << "] is outside the mechanical state. Falling back to initial frame.";
            return false;
        }

        if (startIndex == endIndex)
        {
            msg_warning() << "tangentStartIndex and tangentEndIndex are identical. Falling back to initial frame.";
            return false;
        }

        if (startIndex == constrainedIndex || endIndex == constrainedIndex)
            msg_warning() << "Tangent window includes the constrained index " << constrainedIndex
                          << ". This can create feedback. Prefer using free downstream nodes.";

        Vec3 initialTangent;
        Vec3 currentTangent;
        initialTangent.clear();
        currentTangent.clear();

        for (unsigned int c = 0; c < 3; ++c)
        {
            initialTangent[c] = m_initialPositions[endIndex].getCenter()[c] - m_initialPositions[startIndex].getCenter()[c];
            currentTangent[c] = currentPositions[endIndex].getCenter()[c] - currentPositions[startIndex].getCenter()[c];
        }

        if (norm(initialTangent) <= Real(1e-12) || norm(currentTangent) <= Real(1e-12))
        {
            msg_warning() << "Degenerate tangent window [" << startIndex << ", " << endIndex
                          << "]. Falling back to initial frame.";
            return false;
        }

        // Relative geometric tangent transport convention:
        //     t0   = normalize(x_initial[end] - x_initial[start])
        //     tcur = normalize(x_current[end] - x_current[start])
        //     qDelta = shortestRotation(t0 -> tcur)
        //     qBase  = qDelta * qControlledInitial
        // This uses positions only; no downstream Rigid3 orientation is copied directly.
        tangentDelta = shortestArcQuaternion(initialTangent, currentTangent);
        tangentDelta.normalize();

        usedStartIndex = startIndex;
        usedEndIndex = endIndex;
        return true;
    }
}

template<class DataTypes>
bool ParameterizedFixedProjectiveConstraint<DataTypes>::storePendingTangentDeltaFromState(
    const VecCoord& currentPositions,
    sofa::Index constrainedIndex,
    const char* source)
{
    if constexpr (!std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
    {
        SOFA_UNUSED(currentPositions);
        SOFA_UNUSED(constrainedIndex);
        SOFA_UNUSED(source);
        return false;
    }
    else
    {
        Quat tangentDelta = identityQuaternion();
        sofa::Index usedStartIndex = static_cast<sofa::Index>(-1);
        sofa::Index usedEndIndex = static_cast<sofa::Index>(-1);
        const bool ok = computeTangentDeltaFromState(
            currentPositions,
            constrainedIndex,
            tangentDelta,
            usedStartIndex,
            usedEndIndex);

        if (!ok)
            return false;

        m_pendingTangentDelta = tangentDelta;
        m_pendingTangentDelta.normalize();
        m_pendingTangentDeltaValid = true;
        m_pendingConstrainedIndex = constrainedIndex;
        m_pendingTangentStartIndex = usedStartIndex;
        m_pendingTangentEndIndex = usedEndIndex;

        msg_info() << "Observed pending geometric tangent frame from nodes "
                   << usedStartIndex << " -> " << usedEndIndex
                   << " for constrained index " << constrainedIndex
                   << " in " << source
                   << " qDelta=[" << m_pendingTangentDelta[0] << ", " << m_pendingTangentDelta[1]
                   << ", " << m_pendingTangentDelta[2] << ", " << m_pendingTangentDelta[3]
                   << "]. It will be used after the next reinit().";

        return true;
    }
}

template<class DataTypes>
bool ParameterizedFixedProjectiveConstraint<DataTypes>::refreshActiveTangentDeltaFromState(
    const VecCoord& currentPositions,
    sofa::Index constrainedIndex,
    const char* source)
{
    if constexpr (!std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
    {
        SOFA_UNUSED(currentPositions);
        SOFA_UNUSED(constrainedIndex);
        SOFA_UNUSED(source);
        return false;
    }
    else
    {
        Quat tangentDelta = identityQuaternion();
        sofa::Index usedStartIndex = static_cast<sofa::Index>(-1);
        sofa::Index usedEndIndex = static_cast<sofa::Index>(-1);
        const bool ok = computeTangentDeltaFromState(
            currentPositions,
            constrainedIndex,
            tangentDelta,
            usedStartIndex,
            usedEndIndex);

        if (!ok)
        {
            m_cachedTangentDelta = identityQuaternion();
            m_cachedTangentDeltaValid = false;
            m_cachedConstrainedIndex = constrainedIndex;
            m_cachedTangentStartIndex = usedStartIndex;
            m_cachedTangentEndIndex = usedEndIndex;
            return false;
        }

        m_cachedTangentDelta = tangentDelta;
        m_cachedTangentDelta.normalize();
        m_cachedTangentDeltaValid = true;
        m_cachedConstrainedIndex = constrainedIndex;
        m_cachedTangentStartIndex = usedStartIndex;
        m_cachedTangentEndIndex = usedEndIndex;

        msg_info() << "Refreshed active geometric tangent frame from nodes "
                   << usedStartIndex << " -> " << usedEndIndex
                   << " for constrained index " << constrainedIndex
                   << " in " << source
                   << " qDelta=[" << m_cachedTangentDelta[0] << ", " << m_cachedTangentDelta[1]
                   << ", " << m_cachedTangentDelta[2] << ", " << m_cachedTangentDelta[3]
                   << "].";

        return true;
    }
}

template<class DataTypes>
bool ParameterizedFixedProjectiveConstraint<DataTypes>::refreshActiveTangentCacheFromCurrentState(
    const char* source)
{
    if constexpr (!std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
    {
        SOFA_UNUSED(source);
        return false;
    }
    else
    {
        if (this->mstate.get() == nullptr || frameMode() != FrameMode::Tangent || m_initialPositions.empty())
            return false;

        const VecCoord currentPositions = this->mstate->read(sofa::core::vec_id::read_access::position)->getValue();
        if (currentPositions.empty())
            return false;

        sofa::Index constrainedIndex = 0;
        if (!this->d_fixAll.getValue())
        {
            const auto& indices = this->d_indices.getValue();
            if (indices.empty())
                return false;
            constrainedIndex = indices[0];
        }

        return refreshActiveTangentDeltaFromState(currentPositions, constrainedIndex, source);
    }
}

template<class DataTypes>
bool ParameterizedFixedProjectiveConstraint<DataTypes>::updateTangentDeltaCacheIfNeeded(
    const VecCoord& currentPositions,
    sofa::Index constrainedIndex)
{
    if constexpr (!std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
    {
        SOFA_UNUSED(currentPositions);
        SOFA_UNUSED(constrainedIndex);
        return false;
    }
    else
    {
        if (frameMode() != FrameMode::Tangent)
        {
            m_cachedTangentDelta = identityQuaternion();
            m_cachedTangentDeltaValid = false;
            return false;
        }

        const int startInt = d_tangentStartIndex.getValue();
        const int endInt = d_tangentEndIndex.getValue();
        const sofa::Index requestedStart = startInt >= 0 ? static_cast<sofa::Index>(startInt) : static_cast<sofa::Index>(-1);
        const sofa::Index requestedEnd = endInt >= 0 ? static_cast<sofa::Index>(endInt) : static_cast<sofa::Index>(-1);

        const bool sameActiveCache =
            m_cachedTangentDeltaValid &&
            m_cachedConstrainedIndex == constrainedIndex &&
            m_cachedTangentStartIndex == requestedStart &&
            m_cachedTangentEndIndex == requestedEnd;

        if (sameActiveCache)
            return true;

        // Bootstrap only. This should happen on the first solve or if the tangent window changes.
        // Later projectPosition() calls in the same solve must not replace this active frame.
        return refreshActiveTangentDeltaFromState(currentPositions, constrainedIndex, "projectPosition bootstrap");
    }
}
template<class DataTypes>
typename ParameterizedFixedProjectiveConstraint<DataTypes>::CommandFrame
ParameterizedFixedProjectiveConstraint<DataTypes>::computeCommandFrame(
    const Coord& initialControlled,
    const sofa::type::vector<SReal>& poseOffset,
    sofa::Index parameterBase,
    const Quat& tangentDelta,
    bool tangentDeltaValid) const
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
        SOFA_UNUSED(tangentDelta);
        SOFA_UNUSED(tangentDeltaValid);
        return frame;
    }
    else
    {
        frame.tLocal = parameterTranslation(poseOffset, parameterBase);
        frame.qOffset = parameterRotation(poseOffset, parameterBase);

        Quat qInitialControlled = initialControlled.getOrientation();
        qInitialControlled.normalize();

        if (frameMode() == FrameMode::Tangent && tangentDeltaValid)
        {
            // Tangent mode is a controller-style state-dependent frame:
            //     qBase = qTangentDelta * qControlledInitial
            // The tangent delta is snapshotted once after reinit() and then treated as a stop-gradient frame.
            // We map only the direct derivative with respect to poseOffset.
            frame.qBase = tangentDelta * qInitialControlled;
        }
        else
        {
            // Initial mode:
            //     qBase = qControlledInitial
            frame.qBase = qInitialControlled;
        }

        frame.qBase.normalize();

        // Same convention as the original component:
        //     qOffset is a local/right-multiplied incremental rotation built from poseOffset[3:6].
        // qOffset is deliberately included in RCommand below, so rotation changes insertion direction too:
        //     centerTarget = centerInitial + R(qBase * qOffset) * [tx ty tz].
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
    sofa::Index parameterBase,
    const Quat& tangentDelta,
    bool tangentDeltaValid) const
{
    target = initialControlled;

    if constexpr (std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
    {
        const CommandFrame frame = computeCommandFrame(
            initialControlled,
            poseOffset,
            parameterBase,
            tangentDelta,
            tangentDeltaValid);

        const Vec3 tGlobal = matVec(frame.RCommand, frame.tLocal);

        for (unsigned int i = 0; i < 3; ++i)
            target.getCenter()[i] += tGlobal[i];

        // target.getOrientation() = frame.qCommand;
        target.getOrientation() = initialControlled.getOrientation() * frame.qOffset;
        target.getOrientation().normalize();

        msg_info() << "Command frame"
                   << " | mode=" << d_frameMode.getValue()
                   << " | local x/global=[" << frame.RCommand[0][0] << ", " << frame.RCommand[1][0] << ", " << frame.RCommand[2][0] << "]"
                   << " | tLocal=[" << frame.tLocal[0] << ", " << frame.tLocal[1] << ", " << frame.tLocal[2] << "]"
                   << " | tGlobal=[" << tGlobal[0] << ", " << tGlobal[1] << ", " << tGlobal[2] << "]";
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
    sofa::Index parameterBase,
    const Quat& tangentDelta,
    bool tangentDeltaValid) const
{
    if constexpr (!std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
    {
        SOFA_UNUSED(initialControlled);
        SOFA_UNUSED(poseOffset);
        SOFA_UNUSED(parameterBase);
        SOFA_UNUSED(tangentDelta);
        SOFA_UNUSED(tangentDeltaValid);
        return rawGradient;
    }
    else
    {
        const CommandFrame frame = computeCommandFrame(
            initialControlled,
            poseOffset,
            parameterBase,
            tangentDelta,
            tangentDeltaValid);

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

        // RCommand maps local command coordinates to global coordinates:
        //     vGlobal = RCommand * vLocal
        // Therefore parameter gradients are pulled back by RCommand^T:
        //     gLocal = RCommand^T * gGlobal.
        const Vec3 mappedTranslation = transposeMatVec(frame.RCommand, rawTranslation);
        Vec3 mappedRotation = transposeMatVec(frame.RCommand, rawRotation);

        // Direct translation/rotation coupling is always included because forward projection uses
        //     centerTarget = centerInitial + R(qCommand) * tLocal.
        // For a small local rotation perturbation dtheta:
        //     dCenter = R(qCommand) * (dtheta x tLocal)
        // hence:
        //     dL/dtheta += tLocal x (RCommand^T * dL/dCenter).
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

    // Snapshot the current state before opening the write accessor.
    // In tangent mode, the first projectPosition() after reinit() caches qDelta; repeated
    // Newton/projection calls then reuse that same frame instead of reading the live tangent again.
    const VecCoord currentPositions = xData.getValue();
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

        const bool tangentDeltaValid = updateTangentDeltaCacheIfNeeded(currentPositions, index);
        const Quat tangentDelta = tangentDeltaValid ? m_cachedTangentDelta : identityQuaternion();

        Coord target;
        applyOffsetToCoord(target, m_initialPositions[index], poseOffset, parameterBase, tangentDelta, tangentDeltaValid);
        x[index] = target;

        // Observe the live tangent state for the NEXT solve only. Do not use this pending frame
        // immediately, otherwise the constrained node and tangent node feed each other inside Newton.
        storePendingTangentDeltaFromState(currentPositions, index, "projectPosition");
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
void ParameterizedFixedProjectiveConstraint<DataTypes>::applyParametersJacobianTranspose(const sofa::core::MechanicalParams* mparams,sofa::core::MultiVecDerivId vecId)
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

        bool tangentDeltaValid = false;
        Quat tangentDelta = identityQuaternion();

        if constexpr (std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
        {
            if (frameMode() == FrameMode::Tangent)
            {
                // The adjoint pass must use the same tangent frame as the forward projection.
                // Normally this cache was produced in projectPosition(). If not, refresh once as a fallback.
                if (!m_cachedTangentDeltaValid ||
                    m_cachedConstrainedIndex != constrainedBlock.constrainedIndex)
                {
                    const VecCoord currentPositions = state->read(sofa::core::vec_id::read_access::position)->getValue();
                    updateTangentDeltaCacheIfNeeded(currentPositions, constrainedBlock.constrainedIndex);
                }

                tangentDeltaValid = m_cachedTangentDeltaValid;
                tangentDelta = tangentDeltaValid ? m_cachedTangentDelta : identityQuaternion();
            }
        }

        const VectorN mappedGradient = mapRawGradientToParameterGradient(
            rawGradient,
            m_initialPositions[constrainedBlock.constrainedIndex],
            poseOffset,
            poseOffsetBase,
            tangentDelta,
            tangentDeltaValid);

        for (unsigned int k = 0; k < N; ++k)
        {
            if (!isParameterActive(k))
                continue;

            poseOffsetGradient[poseOffsetBase + k] += mappedGradient[k];
        }
    }

    // After the gradient has been mapped with the active forward frame, observe the current
    // tangent state for the NEXT optimization step. This captures the contact-updated orientation
    // without changing the frame used by the current adjoint computation.
    if constexpr (std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
    {
        if (frameMode() == FrameMode::Tangent && !m_storedBlocks.empty())
        {
            const VecCoord currentPositions = state->read(sofa::core::vec_id::read_access::position)->getValue();
            for (const StoredConstrainedBlock& constrainedBlock : m_storedBlocks)
                storePendingTangentDeltaFromState(currentPositions, constrainedBlock.constrainedIndex, "applyParametersJacobianTranspose");
        }
    }
}

void registerParameterizedFixedProjectiveConstraint(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData(
            "Parameterized FixedProjectiveConstraint with trainable local/tangent poseOffset for Vec3d and Rigid3d.")
        .add<ParameterizedFixedProjectiveConstraint<sofa::defaulttype::Vec3Types>>()
        .add<ParameterizedFixedProjectiveConstraint<sofa::defaulttype::Rigid3Types>>()
    );
}

template class ParameterizedFixedProjectiveConstraint<sofa::defaulttype::Vec3Types>;
template class ParameterizedFixedProjectiveConstraint<sofa::defaulttype::Rigid3Types>;

} // namespace sofadiff
