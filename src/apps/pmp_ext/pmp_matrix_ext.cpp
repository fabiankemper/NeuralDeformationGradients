// =====================================================================================================================

#include "pmp_matrix_ext.h"

// =====================================================================================================================

NAMESPACE_BEGIN(pmp_ext)

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

auto mat3_to_eigen3f(const pmp::Mat3<Scalar>& matrix) -> Eigen::Matrix3f
{
    Eigen::Matrix3f result;

    result <<
        matrix(0, 0), matrix(0, 1), matrix(0, 2),
        matrix(1, 0), matrix(1, 1), matrix(1, 2),
        matrix(2, 0), matrix(2, 1), matrix(2, 2);

    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto eigen3f_to_mat4(const Eigen::Matrix3f& matrix) -> pmp::Mat4<Scalar>
{
    pmp::Mat4<Scalar> result;

    result(0, 0) = matrix(0, 0);
    result(0, 1) = matrix(0, 1);
    result(0, 2) = matrix(0, 2);
    result(0, 3) = 0.0f;

    result(1, 0) = matrix(1, 0);
    result(1, 1) = matrix(1, 1);
    result(1, 2) = matrix(1, 2);
    result(1, 3) = 0.0f;

    result(2, 0) = matrix(2, 0);
    result(2, 1) = matrix(2, 1);
    result(2, 2) = matrix(2, 2);
    result(2, 3) = 0.0f;

    result(3, 0) = 0.0f;
    result(3, 1) = 0.0f;
    result(3, 2) = 0.0f;
    result(3, 3) = 1.0f;

    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto mat3_to_mat4(const pmp::Mat3<Scalar>& matrix) -> pmp::Mat4<Scalar>
{
    pmp::Mat4<Scalar> result;

    result(0, 0) = matrix(0, 0);
    result(0, 1) = matrix(0, 1);
    result(0, 2) = matrix(0, 2);
    result(0, 3) = 0.0f;

    result(1, 0) = matrix(1, 0);
    result(1, 1) = matrix(1, 1);
    result(1, 2) = matrix(1, 2);
    result(1, 3) = 0.0f;

    result(2, 0) = matrix(2, 0);
    result(2, 1) = matrix(2, 1);
    result(2, 2) = matrix(2, 2);
    result(2, 3) = 0.0f;

    result(3, 0) = 0.0f;
    result(3, 1) = 0.0f;
    result(3, 2) = 0.0f;
    result(3, 3) = 1.0f;

    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto mat4_translation_part(const pmp::Mat4<Scalar>& matrix) -> Point
{
    return Point(matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

// ---------------------------------------------------------------------------------------------------------------------

auto mat4_rotation_part(const pmp::Mat4<Scalar>& matrix) -> pmp::Mat3<Scalar>
{
    return pmp::Mat3<Scalar>{
        pmp::Vector<Scalar, 3>(matrix(0, 0), matrix(1, 0), matrix(2, 0)),
        pmp::Vector<Scalar, 3>(matrix(0, 1), matrix(1, 1), matrix(2, 1)),
        pmp::Vector<Scalar, 3>(matrix(0, 2), matrix(1, 2), matrix(2, 2))
    };
}

// ---------------------------------------------------------------------------------------------------------------------

auto mat4_set_translation(const pmp::Mat4<Scalar>& matrix, const pmp::Point& point) -> pmp::Mat4<Scalar>
{
    return pmp::Mat4<Scalar>{
        matrix(0, 0), matrix(0, 1), matrix(0, 2), point[0],
        matrix(1, 0), matrix(1, 1), matrix(1, 2), point[1],
        matrix(2, 0), matrix(2, 1), matrix(2, 2), point[2],
        0.0, 0.0, 0.0, 1.0,
    };
}

// ---------------------------------------------------------------------------------------------------------------------

auto mat4_set_translation_inplace(pmp::Mat4<Scalar>& matrix, const pmp::Point& point) -> void
{
    matrix(0, 3) = point[0];
    matrix(1, 3) = point[1];
    matrix(2, 3) = point[2];
}

// ---------------------------------------------------------------------------------------------------------------------

auto mat4_to_euler_angles(const pmp::Mat4<Scalar>& matrix) -> pmp::vec3
{
    auto rot_3x3 = pmp_ext::mat4_rotation_part(matrix);
    return mat3_to_euler_angles(rot_3x3);
}

// ---------------------------------------------------------------------------------------------------------------------

auto euler_angles_to_mat4(const pmp::vec3& euler_angles) -> pmp::Mat4<Scalar>
{
    return eigen3f_to_mat4(euler_angles_to_mat3(euler_angles));
}

// ---------------------------------------------------------------------------------------------------------------------

auto mat3_to_euler_angles(const pmp::Mat3<Scalar>& matrix) -> pmp::vec3
{
    Eigen::Matrix3f rot_eigen = pmp_ext::mat3_to_eigen3f(matrix);
    // z y x order
    // return rot_eigen.eulerAngles(2, 1, 0);
    // x y z order
    return rot_eigen.eulerAngles(0, 1, 2);
}

// ---------------------------------------------------------------------------------------------------------------------

auto euler_angles_to_mat3(const pmp::vec3& euler_angles) -> pmp::Mat3<Scalar>
{
    Eigen::Matrix3d eigen_recon;
    // z y x order
    // eigen_recon = Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ()) *
    //               Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY()) *
    //               Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitX());
    // x y z order
    eigen_recon = Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ());
    return eigen_recon;
}

// ---------------------------------------------------------------------------------------------------------------------

auto rodrigues_to_mat4(const pmp::vec3& rodrigues) -> pmp::Mat4<Scalar>
{
    return mat3_to_mat4(rodrigues_to_mat3(rodrigues));
}

// ---------------------------------------------------------------------------------------------------------------------

auto mat4_to_rodrigues(const pmp::Mat4<Scalar>& matrix) -> pmp::vec3
{
    return mat3_to_rodrigues(mat4_rotation_part(matrix));
}

// ---------------------------------------------------------------------------------------------------------------------

auto rodrigues_to_mat3(const pmp::vec3& rodriguez) -> pmp::Mat3<Scalar>
{
    const Eigen::Vector3d rodriguez_eigen = rodriguez;
    const double theta = rodriguez_eigen.norm();
    if (theta <= 1e-8) {
        return pmp::Mat3<Scalar>::identity();
    }
    const Eigen::Vector3d axis = rodriguez_eigen.normalized();
    const Eigen::AngleAxisd angle_axis(theta, axis);
    const Eigen::Matrix3d rotation_matrix = angle_axis.matrix();

    return rotation_matrix.cast<float>();
}

// ---------------------------------------------------------------------------------------------------------------------

auto mat3_to_rodrigues(const pmp::Mat3<Scalar>& rotation_matrix) -> pmp::vec3
{
    Eigen::Matrix3d rotation_eigen = pmp_ext::mat3_to_eigen3f(rotation_matrix).cast<double>();
    if (rotation_eigen.isApprox(rotation_eigen.transpose() * rotation_eigen, 1e-6)) {
        rotation_eigen = Eigen::Matrix3d::Identity();
    }

    if (std::abs(rotation_eigen.determinant() - 1.0) >= 1e-6) {

        const Eigen::JacobiSVD<Eigen::Matrix3d> svd(rotation_eigen, Eigen::ComputeFullU | Eigen::ComputeFullV);
        const Eigen::Matrix3d& U = svd.matrixU();
        const Eigen::Matrix3d& V = svd.matrixV();

        Eigen::Matrix3d W = Eigen::Matrix3d::Identity();
        if ((U * V.transpose()).determinant() < 0.0) {
            W(2, 2) = -1.0;
        }
        rotation_eigen = U * W * V.transpose();
    }

    Eigen::AngleAxisd angle_axis(rotation_eigen);
    const double theta = angle_axis.angle();
    const Eigen::Vector3d axis = angle_axis.axis();
    const Eigen::Vector3d rodrigues_vector = theta * axis;
    return pmp::vec3 { rodrigues_vector };
}

// ---------------------------------------------------------------------------------------------------------------------

auto mat4_to_affine(const pmp::Mat4<Scalar>& matrix) -> Eigen::Affine3f
{
    Eigen::Affine3f affine;
    affine.matrix() = matrix;
    return affine;
}

// ---------------------------------------------------------------------------------------------------------------------

auto affine_to_mat4(const Eigen::Affine3f& affine) -> pmp::Mat4<Scalar>
{
    const pmp::Mat4<Scalar> matrix = affine.matrix();
    return matrix;
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(pmp_ext)

// =====================================================================================================================
