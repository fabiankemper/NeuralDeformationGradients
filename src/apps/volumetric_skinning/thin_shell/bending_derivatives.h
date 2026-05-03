//
// Created by gerrit on 6/13/25.
//

#ifndef BENDING_ENERGY_DYNAMIC_DEFORMABLES_H
#define BENDING_ENERGY_DYNAMIC_DEFORMABLES_H
#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>

namespace volumetric_skinning::dihedral {
using V3 = Eigen::Vector3d;

//==============================================================================
// Arctan based implementation of the bending energy. Implements
// the Pseudocode of the 2020 Siggraph course by Theodore Kim.
//==============================================================================

namespace detail {

struct Data {
  double theta;
  double thetaDist;
  std::array<V3, 4> dTheta;
};

inline Data preprocess(const std::array<V3, 4> &p, double theta_rest) {
  /*
   * This is basically taken straight from the book
   */
  constexpr double eps = 1e-12;

  const V3 p01 = p[1] - p[0];
  const V3 p02 = p[2] - p[0];
  const V3 p12 = p[2] - p[1];
  const V3 p31 = p[1] - p[3];
  const V3 p32 = p[2] - p[3];

  V3 n0 = p02.cross(p01);
  V3 n1 = p31.cross(p32);
  double n0Mag = std::max(n0.norm(), eps);
  double n1Mag = std::max(n1.norm(), eps);
  V3 n0hat = n0 / n0Mag;
  V3 n1hat = n1 / n1Mag;
  V3 ehat = p12.normalized();

  double sinTheta = std::clamp((n0hat.cross(n1hat)).dot(ehat), -1.0, 1.0);
  double cosTheta = std::clamp(n0hat.dot(n1hat), -1.0, 1.0);
  double theta = std::atan2(sinTheta, cosTheta);
  double thetaDist = theta - theta_rest;

  auto projectOut = [](const V3 &a, const V3 &b) -> V3 {
    return a - a.dot(b) * b;
  };

  const double invN0 = 1.0 / n0Mag;
  const double invN1 = 1.0 / n1Mag;

  const V3 projN0OutN1 = projectOut(n1hat, n0hat);
  const V3 projN1OutN0 = projectOut(n0hat, n1hat);

  std::array<V3, 4> dCos{};
  dCos[0] = -invN0 * p12.cross(projN0OutN1);

  dCos[1] = -invN0 * (-p02).cross(projN0OutN1);
  dCos[1] += -invN1 * (p32).cross(projN1OutN0);

  dCos[2] = -invN0 * (p01).cross(projN0OutN1);
  dCos[2] += -invN1 * (-p31).cross(projN1OutN0);

  dCos[3] = -invN1 * (-p12).cross(projN1OutN0);

  const V3 n0xe = n0hat.cross(ehat);
  const V3 n1xe = n1hat.cross(ehat);
  const V3 projOutN1Dir = projectOut(n0xe, n1hat);
  const V3 projOutN0Dir = projectOut(n1xe, n0hat);

  std::array<V3, 4> dSin{};
  dSin[0] = -invN0 * p12.cross(projOutN0Dir);

  dSin[1] = invN1 * p32.cross(projOutN1Dir);
  dSin[1] += -invN0 * (-p02).cross(projOutN0Dir);

  dSin[2] = invN1 * (-p31).cross(projOutN1Dir);
  dSin[2] += -invN0 * (p01).cross(projOutN0Dir);

  dSin[3] = invN1 * (-p12).cross(projOutN1Dir);

  std::array<V3, 4> dTheta{};
  for (int i = 0; i < 4; ++i)
    dTheta[i] = cosTheta * dSin[i] - sinTheta * dCos[i];

  return {theta, thetaDist, dTheta};
}
}

inline double energy(const std::array<V3, 4> &p, double theta_rest) {
  const auto d = detail::preprocess(p, theta_rest);
  return 0.5 * d.thetaDist * d.thetaDist;
}

inline double theta(const std::array<V3, 4> &p) {
  const auto d = detail::preprocess(p, 0);
  return d.theta;
}

inline Eigen::Vector<double, 12> gradient(const std::array<V3, 4> &p,
                                          double theta_rest) {
  const auto d = detail::preprocess(p, theta_rest);
  Eigen::Vector<double, 12> g(12);
  for (int i = 0; i < 4; ++i)
    g.segment<3>(i * 3) = -d.thetaDist * d.dTheta[i];
  return g;
}

inline Eigen::Matrix<double, 12, 12> hessian(const std::array<V3, 4> &p,
                                             double theta_rest) {
  /*
   *  This is not actually the hessian, just a Gauss-Newton type approximation
   *  using the G^TG. P.S.D. by Construction as recommended in the SIGGRAPH COURSE
   *  of Kim and Eberle
   */
  const auto d = detail::preprocess(p, theta_rest);
  Eigen::Vector<double, 12> g(12);
  for (int i = 0; i < 4; ++i)
    g.segment<3>(i * 3) = d.dTheta[i];
  return g * g.transpose();
}

} // namespace volumetric_skinning::dihedral
#endif // BENDING_ENERGY_DYNAMIC_DEFORMABLES_H
