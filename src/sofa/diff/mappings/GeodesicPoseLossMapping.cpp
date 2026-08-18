#include <sofa/diff/mappings/GeodesicPoseLossMapping.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/accessor.h>

#include <algorithm>
#include <cmath>
#include <type_traits>

namespace sofadiff
{

template<class DataTypes>
GeodesicPoseLossMapping<DataTypes>::GeodesicPoseLossMapping()
    : Inherit()
    , d_targetPose(this->initData(
          &d_targetPose,
          sofa::type::vector<SReal>{},
          "targetPose",
          "Target pose. Rigid3d: [x y z qx qy qz qw]. Vec6d: [x y z rx ry rz]."))
    , d_positionWeight(this->initData(
          &d_positionWeight,
          static_cast<SReal>(1.0),
          "positionWeight",
          "Weight of the position term."))
    , d_rotationWeight(this->initData(
          &d_rotationWeight,
          static_cast<SReal>(1.0),
          "rotationWeight",
          "Weight of the geodesic rotation term."))
{
}

template<class DataTypes>
sofa::Index GeodesicPoseLossMapping<DataTypes>::targetStride() const
{
    if constexpr (std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
        return 7;

    return N;
}

template<class DataTypes>
sofa::Index GeodesicPoseLossMapping<DataTypes>::targetBaseForIndex(
    sofa::Index index,
    sofa::Size inputSize,
    sofa::Size targetSize) const
{
    const sofa::Index stride = targetStride();

    if (targetSize >= stride * inputSize)
        return stride * index;

    return 0;
}

template<class DataTypes>
sofa::type::Vec<3, typename GeodesicPoseLossMapping<DataTypes>::Real>
GeodesicPoseLossMapping<DataTypes>::positionOf(const Coord& coord) const
{
    sofa::type::Vec<3, Real> p;

    if constexpr (std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
    {
        p[0] = coord.getCenter()[0];
        p[1] = coord.getCenter()[1];
        p[2] = coord.getCenter()[2];
    }
    else
    {
        p[0] = coord[0];
        p[1] = coord[1];
        p[2] = coord[2];
    }

    return p;
}

template<class DataTypes>
sofa::type::Vec<3, typename GeodesicPoseLossMapping<DataTypes>::Real>
GeodesicPoseLossMapping<DataTypes>::targetPosition(
    const sofa::type::vector<SReal>& target,
    sofa::Index base) const
{
    sofa::type::Vec<3, Real> p;
    p[0] = static_cast<Real>(target[base + 0]);
    p[1] = static_cast<Real>(target[base + 1]);
    p[2] = static_cast<Real>(target[base + 2]);
    return p;
}

template<class DataTypes>
sofa::type::Vec<3, typename GeodesicPoseLossMapping<DataTypes>::Real>
GeodesicPoseLossMapping<DataTypes>::rotationError(
    const Coord& coord,
    const sofa::type::vector<SReal>& target,
    sofa::Index base) const
{
    sofa::type::Vec<3, Real> error;
    error[0] = Real(0);
    error[1] = Real(0);
    error[2] = Real(0);

    if constexpr (std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types>)
    {
        const auto& q = coord.getOrientation();

        Real qx = q[0];
        Real qy = q[1];
        Real qz = q[2];
        Real qw = q[3];

        Real tx = static_cast<Real>(target[base + 3]);
        Real ty = static_cast<Real>(target[base + 4]);
        Real tz = static_cast<Real>(target[base + 5]);
        Real tw = static_cast<Real>(target[base + 6]);

        const Real targetNorm =
            std::sqrt(tx * tx + ty * ty + tz * tz + tw * tw);

        if (targetNorm > Real(1e-12))
        {
            tx /= targetNorm;
            ty /= targetNorm;
            tz /= targetNorm;
            tw /= targetNorm;
        }
        else
        {
            tx = Real(0);
            ty = Real(0);
            tz = Real(0);
            tw = Real(1);
        }

        const Real ex =  tw * qx - tx * qw - ty * qz + tz * qy;
        const Real ey =  tw * qy + tx * qz - ty * qw - tz * qx;
        const Real ez =  tw * qz - tx * qy + ty * qx - tz * qw;
        Real ew       =  tw * qw + tx * qx + ty * qy + tz * qz;

        Real vx = ex;
        Real vy = ey;
        Real vz = ez;

        if (ew < Real(0))
        {
            vx = -vx;
            vy = -vy;
            vz = -vz;
            ew = -ew;
        }

        const Real vNorm = std::sqrt(vx * vx + vy * vy + vz * vz);

        if (vNorm < Real(1e-12))
        {
            error[0] = Real(2) * vx;
            error[1] = Real(2) * vy;
            error[2] = Real(2) * vz;
        }
        else
        {
            const Real angle = Real(2) * std::atan2(vNorm, ew);
            const Real scale = angle / vNorm;

            error[0] = scale * vx;
            error[1] = scale * vy;
            error[2] = scale * vz;
        }
    }
    else
    {
        if constexpr (N >= 6)
        {
            error[0] = coord[3] - static_cast<Real>(target[base + 3]);
            error[1] = coord[4] - static_cast<Real>(target[base + 4]);
            error[2] = coord[5] - static_cast<Real>(target[base + 5]);
        }
    }

    return error;
}

template<class DataTypes>
void GeodesicPoseLossMapping<DataTypes>::apply(
    const sofa::core::MechanicalParams* mparams,
    OutDataVecCoord& out,
    const InDataVecCoord& in)
{
    SOFA_UNUSED(mparams);

    const auto input = sofa::helper::getReadAccessor(in);
    auto output = sofa::helper::getWriteAccessor(out);

    const auto& target = d_targetPose.getValue();

    const sofa::Size inputSize = input.size();
    const sofa::Index stride = targetStride();

    if (inputSize == 0 || target.size() < stride)
    {
        output[0][0] = SReal(0);
        m_inputGradient.clear();
        return;
    }

    const SReal wp = d_positionWeight.getValue();
    const SReal wr = d_rotationWeight.getValue();

    SReal loss = SReal(0);
    m_inputGradient.resize(inputSize);

    for (sofa::Index i = 0; i < inputSize; ++i)
    {
        Deriv gradient;
        for (unsigned int c = 0; c < N; ++c)
            gradient[c] = Real(0);

        const sofa::Index base =
            targetBaseForIndex(i, inputSize, target.size());

        const auto p = positionOf(input[i]);
        const auto pt = targetPosition(target, base);

        for (unsigned int c = 0; c < 3; ++c)
        {
            const Real e = p[c] - pt[c];
            loss += wp * e * e;
            gradient[c] += static_cast<Real>(Real(2) * wp * e);
        }

        if constexpr (std::is_same_v<DataTypes, sofa::defaulttype::Rigid3Types> || N >= 6)
        {
            const auto r = rotationError(input[i], target, base);

            for (unsigned int c = 0; c < 3; ++c)
            {
                loss += wr * r[c] * r[c];

                if constexpr (N >= 6)
                    gradient[3 + c] += static_cast<Real>(Real(2) * wr * r[c]);
            }
        }

        m_inputGradient[i] = gradient;
    }

    output[0][0] = loss / static_cast<SReal>(inputSize);
}

template<class DataTypes>
void GeodesicPoseLossMapping<DataTypes>::applyJ(
    const sofa::core::MechanicalParams* mparams,
    OutDataVecDeriv& out,
    const InDataVecDeriv& in)
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(out);
    SOFA_UNUSED(in);
}

template<class DataTypes>
void GeodesicPoseLossMapping<DataTypes>::applyJT(
    const sofa::core::MechanicalParams* mparams,
    InDataVecDeriv& out,
    const OutDataVecDeriv& in)
{
    SOFA_UNUSED(mparams);

    const auto upstream = sofa::helper::getReadAccessor(in);
    auto result = sofa::helper::getWriteAccessor(out);

    const SReal scalar = upstream[0][0];
    const sofa::Size size = std::min(result.size(), m_inputGradient.size());

    if (size == 0)
        return;

    for (sofa::Index i = 0; i < size; ++i)
    {
        for (unsigned int c = 0; c < N; ++c)
        {
            result[i][c] +=
                scalar * m_inputGradient[i][c] / static_cast<SReal>(size);
        }
    }
}

void registerGeodesicPoseLossMapping(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(
        sofa::core::ObjectRegistrationData(
            "Geodesic pose loss mapping. Rigid3d uses position error plus quaternion geodesic rotation error.")
        .add<GeodesicPoseLossMapping<sofa::defaulttype::Rigid3Types>>()
        .add<GeodesicPoseLossMapping<sofa::defaulttype::Vec6Types>>()
    );
}

template class GeodesicPoseLossMapping<sofa::defaulttype::Rigid3Types>;
template class GeodesicPoseLossMapping<sofa::defaulttype::Vec6Types>;

} // namespace sofadiff