#pragma once

#include <sofa/diff/ParameterizedForceField.h>

#include <sofa/component/constraint/projective/FixedProjectiveConstraint.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/behavior/BaseProjectiveConstraintSet.h>

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/type/vector.h>
#include <sofa/type/Vec.h>
#include <sofa/type/Mat.h>
#include <sofa/type/Quat.h>

#include <array>
#include <type_traits>

namespace sofadiff
{

template<class DataTypes>
class ParameterizedFixedProjectiveConstraint :
    public Parameterized,
    public sofa::component::constraint::projective::FixedProjectiveConstraint<DataTypes>
{
public:
    using Inherit = sofa::component::constraint::projective::FixedProjectiveConstraint<DataTypes>;

    using Real = typename DataTypes::Real;
    using Coord = typename DataTypes::Coord;
    using Deriv = typename DataTypes::Deriv;
    using VecCoord = typename DataTypes::VecCoord;
    using VecDeriv = typename DataTypes::VecDeriv;
    using DataVecCoord = sofa::Data<VecCoord>;

    static constexpr unsigned int N = Deriv::size();

    using VectorN = sofa::type::Vec<N, SReal>;
    using MatrixN = sofa::type::Mat<N, N, SReal>;
    using Vec3 = sofa::type::Vec<3, Real>;
    using Mat3 = sofa::type::Mat<3, 3, Real>;
    using Quat = sofa::type::Quat<Real>;
    using DenseColumnArray = std::array<sofa::type::vector<SReal>, N>;

    SOFA_CLASS(
        SOFA_TEMPLATE(ParameterizedFixedProjectiveConstraint, DataTypes),
        SOFA_TEMPLATE(sofa::component::constraint::projective::FixedProjectiveConstraint, DataTypes)
    );

public:
    ParameterizedFixedProjectiveConstraint();

    void init() override;
    void reinit() override;

    void projectPosition(
        const sofa::core::MechanicalParams* mparams,
        DataVecCoord& xData) override;

    void applyConstraint(
        sofa::core::behavior::ZeroDirichletCondition* matrix) override;

    void applyParametersJacobianTranspose(
        const sofa::core::MechanicalParams* mparams,
        sofa::core::MultiVecDerivId vecId) override;

protected:
    struct CommandFrame
    {
        Quat qBase;
        Quat qOffset;
        Quat qCommand;
        Mat3 RCommand;
        Vec3 tLocal;
    };

    struct StoredCouplingBlock
    {
        sofa::Index nodeIndex {};
        MatrixN stiffness; // Column block K_{nodeIndex, constrainedIndex} captured before Dirichlet deletion.
    };

    struct StoredConstrainedBlock
    {
        sofa::Index constrainedIndex {};
        sofa::type::vector<StoredCouplingBlock> couplingBlocks;
    };

protected:
    void captureInitialPositions();

    sofa::Index parameterBaseForStoredBlock(
        sofa::Index storedBlockId,
        sofa::Size parameterSize) const;

    void applyOffsetToCoord(
        Coord& target,
        const Coord& initialControlled,
        const sofa::type::vector<SReal>& poseOffset,
        sofa::Index parameterBase) const;

    VectorN mapRawGradientToParameterGradient(
        const VectorN& rawGradient,
        const Coord& initialControlled,
        const sofa::type::vector<SReal>& poseOffset,
        sofa::Index parameterBase) const;

    CommandFrame computeCommandFrame(
        const Coord& initialControlled,
        const sofa::type::vector<SReal>& poseOffset,
        sofa::Index parameterBase) const;

    bool isParameterActive(unsigned int component) const;

    bool captureDenseColumns(
        sofa::core::behavior::ZeroDirichletCondition* matrix,
        sofa::Index constrainedIndex,
        sofa::Size matrixSize,
        DenseColumnArray& columns) const;

    void compressDenseColumnsToBlocks(
        const DenseColumnArray& columns,
        sofa::Index constrainedIndex,
        sofa::Size matrixSize,
        StoredConstrainedBlock& outputBlock) const;

    static bool isZeroBlock(const MatrixN& block);

    static Mat3 identityMatrix();
    SReal parameterValue(const sofa::type::vector<SReal>& poseOffset, sofa::Index parameterBase, unsigned int component) const;
    Vec3 parameterTranslation(const sofa::type::vector<SReal>& poseOffset, sofa::Index parameterBase) const;
    Quat parameterRotation(const sofa::type::vector<SReal>& poseOffset, sofa::Index parameterBase) const;
    static Quat identityQuaternion();
    static Mat3 quaternionToMatrix(const Quat& q);
    static Vec3 matVec(const Mat3& A, const Vec3& x);
    static Vec3 transposeMatVec(const Mat3& A, const Vec3& x);
    static Vec3 cross(const Vec3& a, const Vec3& b);

protected:
    sofa::Data<sofa::type::vector<SReal>> d_poseOffset;
    Parameter<sofa::type::vector<SReal>>* m_poseOffsetParameter { nullptr };

    // Active parameter mask. Inactive components are ignored in the forward projection and in the gradient update.
    // This prevents stale values in disabled DOFs from affecting the constrained pose.
    // Rigid3d order: [tx ty tz rx ry rz]. Vec3d order: [x y z].
    sofa::Data<sofa::type::vector<bool>> d_componentFlags;

    // Reference configuration captured at initialization.
    VecCoord m_initialPositions;

    sofa::type::vector<StoredConstrainedBlock> m_storedBlocks;
};

void registerParameterizedFixedProjectiveConstraint(sofa::core::ObjectFactory* factory);

} // namespace sofadiff
