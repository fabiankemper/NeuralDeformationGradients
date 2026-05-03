#include "NeoHookeanEnergyQuadrature.h"
#include "neo_hookean_derivatives.h"
#include "volumetric_base/diffgeo/prism_diffgeo.h"
#include "volumetric_base/utility/matrix_util.hpp"

// 6 Point Quadrature, 3 points for tris, 2 for line

struct QuadPoint {
  double a, b, z; // barycentric α, β, and through-thickness ξ
  double w;       // weight
};

const std::vector<QuadPoint> prismQuadrature() {
  // Triangle (3-pt, area = 0.5)
  const double tri[3][2] = {
      {1.0 / 6.0, 1.0 / 6.0}, {2.0 / 3.0, 1.0 / 6.0}, {1.0 / 6.0, 2.0 / 3.0}};
  const double wtT = 1.0 / 6.0;

  // Line (2-pt Gauss-Legendre on [0,1])
  const double zeta[2] = {0.5 * (1.0 - 1.0 / std::sqrt(3.0)),
                          0.5 * (1.0 + 1.0 / std::sqrt(3.0))};
  const double wtL = 0.5;

  std::vector<QuadPoint> qp;
  qp.reserve(6);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 2; ++j)
      qp.push_back({tri[i][0], tri[i][1], zeta[j], wtT * wtL});

  return qp;
}

const std::vector<QuadPoint> quadrature_points = prismQuadrature();

Eigen::Matrix<double, 3, 6> get_matrix_C(double alpha, double beta,
                                         double zeta) {
  Eigen::Matrix<double, 3, 6> C;
  double gamma = 1 - alpha - beta;
  C << zeta, 0, -zeta, (1 - zeta), 0, -(1 - zeta), 0, zeta, -zeta, 0,
      (1 - zeta), -(1 - zeta), alpha, beta, gamma, -alpha, -beta, -gamma;
  return C;
}

Eigen::Matrix<double, 3, 6>
get_gradient(const Eigen::Matrix<double, 6, 3>
                 &undeformed_positions, // Assume positions as rows, 1 through 6
             double alpha, double beta, double zeta) {
  Eigen::Matrix<double, 3, 6> C = get_matrix_C(alpha, beta, zeta);
  Eigen::Matrix<double, 3, 3> J_undeformed = C * undeformed_positions;
  return J_undeformed.partialPivLu().solve(C);
}

namespace volumetric_skinning {
namespace neo_hookean {

std::array<double, 3>
NeoHookeanEnergyQuadrature::get_material_parameters(const int i) const {
  double mu, lambda;
  if (i % 2 == 0) {
    mu = muscle_lame_mu;
    lambda = muscle_lame_lambda;
  } else {
    mu = skin_lame_mu;
    lambda = skin_lame_lambda;
  }
  return {mu, lambda, 1.0 + mu / lambda};
}

double NeoHookeanEnergyQuadrature::energy(
    const volumetric_base::LayeredMesh &deformed_mesh) {
  double energy = 0;
#pragma omp parallel for reduction(+ : energy)
  for (int i = 0; i < static_cast<int>(prisms.size()); i++) {
    for (unsigned int qp = 0; qp < quadrature_points.size(); qp++) {
      auto [mu, lambda, alpha] = get_material_parameters(i);
      const Eigen::Matrix<double, 6, 3> positions =
          deformed_mesh.getPositionsAsMatrix(prisms[i]).transpose();
      const Eigen::Matrix<double, 3, 6> local_grad =
          gradients[i * quadrature_points.size() + qp];
      const Eigen::Matrix3d F = local_grad * positions;
      energy += weights[i * quadrature_points.size() + qp] *
                stable_neo_hookean::compute_energy(F, mu, lambda, alpha);
    }
  }
  return energy;
}

Eigen::VectorXd NeoHookeanEnergyQuadrature::gradient(
    const volumetric_base::LayeredMesh &deformed_mesh) {
  Eigen::VectorXd global_gradient(num_dofs * 3);
  global_gradient.setZero();

  for (unsigned int i = 0; i < prisms.size(); i++) {
    for (unsigned int k = 0; k < quadrature_points.size(); k++) {

      const Eigen::Matrix<double, 3, 6> local_gradient =
          gradients[i * quadrature_points.size() + k];
      auto [mu, lambda, alpha] = get_material_parameters(i);
      auto [i1, i2, i3, i4, i5, i6] = deformed_mesh.getIndices(prisms[i]);
      const Eigen::Matrix3d F =
          local_gradient *
          deformed_mesh.getPositionsAsMatrix(prisms[i]).transpose();

      const Eigen::Matrix<double, 3, 6> grad =
          weights[i * quadrature_points.size() + k] *
          stable_neo_hookean::gradient(F, mu, lambda, alpha).transpose() *
          local_gradient;

      if (constrained_mapping[i1] != -1)
        global_gradient.block<3, 1>(constrained_mapping[i1] * 3, 0) +=
            grad.col(0);
      if (constrained_mapping[i2] != -1)
        global_gradient.block<3, 1>(constrained_mapping[i2] * 3, 0) +=
            grad.col(1);
      if (constrained_mapping[i3] != -1)
        global_gradient.block<3, 1>(constrained_mapping[i3] * 3, 0) +=
            grad.col(2);
      if (constrained_mapping[i4] != -1)
        global_gradient.block<3, 1>(constrained_mapping[i4] * 3, 0) +=
            grad.col(3);
      if (constrained_mapping[i5] != -1)
        global_gradient.block<3, 1>(constrained_mapping[i5] * 3, 0) +=
            grad.col(4);
      if (constrained_mapping[i6] != -1)
        global_gradient.block<3, 1>(constrained_mapping[i6] * 3, 0) +=
            grad.col(5);
    }
  }

  return global_gradient;
}

void NeoHookeanEnergyQuadrature::setup(volumetric_base::LayeredMesh &mesh,
                                       std::vector<int> constrained_mapping) {
  undeformed_mesh = mesh;
  for (const auto &prism : mesh.getPrisms()) {
    prisms.emplace_back(prism);
    for (const auto [a, b, z, w] : quadrature_points) {
      gradients.emplace_back(
          get_gradient(mesh.getPositionsAsMatrix(prism).transpose(), a, b, z));
      weights.emplace_back(w * std::abs(prism_diffgeo::prism_volume(
                                   mesh.getPositionsAsMatrix(prism))));
    };
  }
  this->constrained_mapping = constrained_mapping;
  num_dofs = 0;
  for (int idx : constrained_mapping) {
    if (idx != -1)
      num_dofs++;
  }
  hessian_constructor = newton::HessianConstructor<6>(constrained_mapping);
}

Eigen::SparseMatrix<double> NeoHookeanEnergyQuadrature::hessian(
    const volumetric_base::LayeredMesh &deformed_mesh,
    [[maybe_unused]] bool &patternAnalyzed) {

  Eigen::MatrixXd positions = deformed_mesh.toVertexMatrix();
  auto local_hess = [&]([[maybe_unused]] std::array<int, 6> prism,
                        int i) -> Eigen::Matrix<double, 18, 18> {
    Eigen::Matrix<double, 18, 18> H;
    H.setZero();
    for (unsigned int k = 0; k < quadrature_points.size(); k++) {
      const Eigen::Matrix<double, 3, 6> local_gradient =
          gradients[i * quadrature_points.size() + k];
      const Eigen::Matrix3d F =
          local_gradient *
          deformed_mesh.getPositionsAsMatrix(prisms[i]).transpose();
      const Eigen::Matrix<double, 9, 18> G =
          matrix_util::vectorize_gradient(local_gradient);

      const auto [mu, lambda, alpha] = get_material_parameters(i);
      // G^T * d^2 PSI(X) / dF^2 * G
      // psd_projection needed for gauss-newton
      H += weights[i * quadrature_points.size() + k] * G.transpose() *
           matrix_util::psd_projection_to_abs(
               stable_neo_hookean::hessian(F, mu, lambda, alpha)) *
           G;
    }
    return H;
  };
  std::vector<std::array<int, 6>> indices;
  for (auto p : prisms) {
    auto [i1, i2, i3, i4, i5, i6] = deformed_mesh.getIndices(p);
    std::array ids = {i1, i2, i3, i4, i5, i6};
    indices.emplace_back(ids);
  }
  return hessian_constructor.assemble_hessian(local_hess, indices);
}

} // namespace neo_hookean

} // namespace volumetric_skinning
