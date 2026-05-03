#ifndef INVARIANTS_AND_DERIVATIVES_H
#define INVARIANTS_AND_DERIVATIVES_H
#include <Eigen/Core>
#include "apps/volumetric_base/utility/matrix_util.hpp"


/*
 *  Contains implementations of elastic invariants
 *  and their gradients/hessians
 */

NAMESPACE_BEGIN(volumetric_skinning::elastic_invariants)

inline Eigen::Matrix3d cross_matrix(const Eigen::Vector3d &f) {
    Eigen::Matrix3d res;
    res << 0    ,-f[2], f[1],
           f[2] , 0   ,-f[0],
          -f[1] , f[0],   0;
    return res;
}

inline double I2(const Eigen::Matrix3d& F) {
    return F.squaredNorm();
}

inline double I3(const Eigen::Matrix3d& F) {
    return F.determinant();
}

inline Eigen::Vector<double,9> I2_grad(const Eigen::Matrix3d& F) {
    return matrix_util::flatten(2*F);
}

inline Eigen::Vector<double,9> I3_grad(const Eigen::Matrix3d& F) {
    const Eigen::Vector3d f0 = F.row(0);
    const Eigen::Vector3d f1 = F.row(1);
    const Eigen::Vector3d f2 = F.row(2);
    Eigen::Vector<double,9> grad;
    grad.segment<3>(0) = f1.cross(f2);
    grad.segment<3>(3) = f2.cross(f0);
    grad.segment<3>(6) = f0.cross(f1);
    return grad;
}

inline Eigen::Matrix<double,9,9> I2_hess([[maybe_unused]] const Eigen::Matrix3d& F) {
    return 2*Eigen::MatrixXd::Identity(9,9);
}


inline Eigen::Matrix<double,9,9> I3_hess(const Eigen::Matrix3d& F) {
    const Eigen::Matrix3d f0_hat = cross_matrix(F.row(0));
    const Eigen::Matrix3d f1_hat = cross_matrix(F.row(1));
    const Eigen::Matrix3d f2_hat = cross_matrix(F.row(2));

    Eigen::Matrix<double,9,9> H;
    H.setZero();
    H.block<3,3>(3,0) = f2_hat;
    H.block<3,3>(0,3) = -f2_hat;

    H.block<3,3>(6,0) = -f1_hat;
    H.block<3,3>(0,6) = f1_hat;

    H.block<3,3>(3,6) = -f0_hat;
    H.block<3,3>(6,3) = f0_hat;
    return H;
}

NAMESPACE_END(volumetric_skinning::elastic_invariants)

#endif //INVARIANTS_AND_DERIVATIVES_H
