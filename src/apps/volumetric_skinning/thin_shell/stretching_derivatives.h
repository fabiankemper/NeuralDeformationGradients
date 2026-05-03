#ifndef THIN_SHELL_DERIVATIVES_H
#define THIN_SHELL_DERIVATIVES_H
#include <Eigen/Core>

#include "pmp/surface_mesh.h"
#include "volanim_utils/defs.h"

NAMESPACE_BEGIN(volumetric_skinning::thin_shells)
//==================================================================================================================
//                          Thin Shell Stretching after Grinspun et al. 2004                                       #
//                                                                                                                 #
//==================================================================================================================

inline double stretching_energy(const Eigen::Vector3d &Pi,
                                const Eigen::Vector3d &Pj, double old_length) {
  double new_length = (Pi - Pj).norm();
  return pow(new_length - old_length, 2) / 2;
}

inline Eigen::Matrix3d vectorNormalizeJacobian(Eigen::VectorXd vector) {
  // tested with FEM, 90%
  Eigen::Matrix3d jacobian;
  double len = vector.norm();
  double len_cubed = len * len * len;
  jacobian(0, 0) = vector[1] * vector[1] + vector[2] * vector[2];
  jacobian(1, 1) = vector[0] * vector[0] + vector[2] * vector[2];
  jacobian(2, 2) = vector[0] * vector[0] + vector[1] * vector[1];
  jacobian(0, 1) = -vector[0] * vector[1];
  jacobian(1, 0) = -vector[0] * vector[1];
  jacobian(0, 2) = -vector[0] * vector[2];
  jacobian(2, 0) = -vector[0] * vector[2];
  jacobian(1, 2) = -vector[1] * vector[2];
  jacobian(2, 1) = -vector[1] * vector[2];
  return jacobian / len_cubed;
}
inline Eigen::Matrix<double, 6, 6> stretching_hessian(Eigen::VectorXd edge,
                                                      double target_length) {
  Eigen::Matrix3d D = vectorNormalizeJacobian(std::move(edge));
  Eigen::Matrix3d id = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d local_hess = (id - target_length * D);
  Eigen::Matrix<double, 6, 6> H;
  H.block<3, 3>(0, 0) = local_hess;
  H.block<3, 3>(3, 3) = local_hess;
  H.block<3, 3>(3, 0) = -local_hess;
  H.block<3, 3>(0, 3) = -local_hess;
  return H;
}

inline Eigen::Vector<double, 6> stretching_gradient(const Eigen::Vector3d &Pi,
                                                    const Eigen::Vector3d &Pj,
                                                    double old_length) {
  Eigen::Vector<double, 6> grad;
  Eigen::Vector3d dist = Pi - Pj;
  double len = dist.norm();
  dist = dist / len;
  double factor = (len - old_length);
  const Eigen::Vector3d x1derivative = dist;
  const Eigen::Vector3d x2derivative = -dist;
  grad.block<3, 1>(0, 0) = factor * x1derivative;
  grad.block<3, 1>(3, 0) = factor * x2derivative;
  return grad;
}

NAMESPACE_END(volumetric_skinning::thin_shells)

#endif // THIN_SHELL_DERIVATIVES_H
