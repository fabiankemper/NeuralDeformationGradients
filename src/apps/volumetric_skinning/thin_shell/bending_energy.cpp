#include "bending_energy.h"

#include "apps/volumetric_base/layered_mesh.h"
#include "bending_derivatives.h"

/**
/                   p2
/                 / | \
/                /  |  \
/               /   |   \
/              p0 n0|n1 p3
/               \   |   /
/                \  |  /
/                 \ | /
/                  p1
**/

std::array<int, 4> get_diamond(const volumetric_base::LayeredMesh &mesh,
                               const pmp::Edge &e) {
  const pmp::Halfedge h = mesh.halfedge(e, 0);
  const pmp::Halfedge h_reverse = mesh.opposite_halfedge(h);

  const pmp::Halfedge next = mesh.next_halfedge(h);
  const pmp::Halfedge next_reverse = mesh.next_halfedge(h_reverse);

  const pmp::Vertex p1 = mesh.from_vertex(h);
  const pmp::Vertex p2 = mesh.to_vertex(h);
  const pmp::Vertex p0 = mesh.to_vertex(next);
  const pmp::Vertex p3 = mesh.to_vertex(next_reverse);
  return {static_cast<int>(volumetric_base::LayeredMesh::Index(
              p0, volumetric_base::Layer::SKIN)),
          static_cast<int>(volumetric_base::LayeredMesh::Index(
              p1, volumetric_base::Layer::SKIN)),
          static_cast<int>(volumetric_base::LayeredMesh::Index(
              p2, volumetric_base::Layer::SKIN)),
          static_cast<int>(volumetric_base::LayeredMesh::Index(
              p3, volumetric_base::Layer::SKIN))};
}

void volumetric_skinning::thin_shells::ThinShellBendingEnergy::setup(
    volumetric_base::LayeredMesh &mesh, std::vector<int> constrained_mapping_) {
  this->constrained_mapping = constrained_mapping_;
  n_active_vertices = 0;
  for (int idx : constrained_mapping_) {
    if (idx != -1)
      n_active_vertices++;
  }
  Eigen::MatrixXd posis = mesh.toVertexMatrix();
  for (const pmp::Edge &edge : mesh.edges()) {
    if (mesh.is_boundary(edge))
      continue;
    std::array<int, 4> diam = get_diamond(mesh, edge);
    diamonds.emplace_back(diam);
    auto [i, j, k, l] = get_diamond(mesh, edge);
    const Eigen::Vector3d p0 = posis.row(i);
    const Eigen::Vector3d p1 = posis.row(j);
    const Eigen::Vector3d p2 = posis.row(k);
    const Eigen::Vector3d p3 = posis.row(l);
    original_angles.emplace_back(0);
    double Le = (p0 - p3).norm();
    double area = ((p1 - p0).cross(p2 - p0)).norm();
    area += ((p1 - p3).cross(p2 - p3)).norm() / 2;
    weights.emplace_back(Le * Le / area / 2);
    hessian_constructor = newton::HessianConstructor<4>(constrained_mapping_);
  }
}

Eigen::VectorXd
volumetric_skinning::thin_shells::ThinShellBendingEnergy::gradient(
    const volumetric_base::LayeredMesh &deformed_mesh) {
  Eigen::VectorXd gradient(n_active_vertices * 3);
  gradient.setZero();
  Eigen::MatrixXd posis = deformed_mesh.toVertexMatrix();
  for (unsigned int diamond_idx = 0; diamond_idx < diamonds.size(); diamond_idx++) {
    const auto [i, j, k, l] = diamonds[diamond_idx];
    const Eigen::Vector3d Pi = posis.row(i);
    const Eigen::Vector3d Pj = posis.row(j);
    const Eigen::Vector3d Pk = posis.row(k);
    const Eigen::Vector3d Pl = posis.row(l);
    Eigen::Vector<double, 12> local_grad =
        weights[diamond_idx] *
        dihedral::gradient({Pi, Pj, Pk, Pl}, original_angles[diamond_idx]);

    if (constrained_mapping[i] != -1)
      gradient.segment<3>(constrained_mapping[i] * 3) +=
          local_grad.segment<3>(0);
    if (constrained_mapping[j] != -1)
      gradient.segment<3>(constrained_mapping[j] * 3) +=
          local_grad.segment<3>(3);
    if (constrained_mapping[k] != -1)
      gradient.segment<3>(constrained_mapping[k] * 3) +=
          local_grad.segment<3>(6);
    if (constrained_mapping[l] != -1)
      gradient.segment<3>(constrained_mapping[l] * 3) +=
          local_grad.segment<3>(9);
  }
  return gradient;
}

Eigen::SparseMatrix<double>
volumetric_skinning::thin_shells::ThinShellBendingEnergy::hessian(
    const volumetric_base::LayeredMesh &deformed_mesh, [[maybe_unused]] bool &patternAnalyzed) {
  Eigen::SparseMatrix<double> H(n_active_vertices * 3, n_active_vertices * 3);
  std::vector<Eigen::Triplet<double>> triplets;
  const Eigen::MatrixXd posis = deformed_mesh.toVertexMatrix();

  auto construct_local_hess =
      [&](std::array<int, 4> diamond,
          int diamond_idx) -> Eigen::Matrix<double, 12, 12> {
    const auto [i, j, k, l] = diamond;
    const Eigen::Vector3d Pi = posis.row(i);
    const Eigen::Vector3d Pj = posis.row(j);
    const Eigen::Vector3d Pk = posis.row(k);
    const Eigen::Vector3d Pl = posis.row(l);
    return weights[diamond_idx] *
           dihedral::hessian({Pi, Pj, Pk, Pl}, original_angles[diamond_idx]);
  };
  return hessian_constructor.assemble_hessian(construct_local_hess, diamonds);
}

double volumetric_skinning::thin_shells::ThinShellBendingEnergy::energy(
    const volumetric_base::LayeredMesh &deformed_mesh) {
  double energy = 0;
  Eigen::MatrixXd posis = deformed_mesh.toVertexMatrix();
#pragma omp parallel for reduction(+ : energy)
  for (int diamond_idx = 0; diamond_idx < static_cast<int> (diamonds.size()); ++diamond_idx) {
    const auto [i, j, k, l] = diamonds[diamond_idx];
    const Eigen::Vector3d Pi = posis.row(i);
    const Eigen::Vector3d Pj = posis.row(j);
    const Eigen::Vector3d Pk = posis.row(k);
    const Eigen::Vector3d Pl = posis.row(l);
    energy += weights[diamond_idx] *
              dihedral::energy({Pi, Pj, Pk, Pl}, original_angles[diamond_idx]);
  }
  return energy;
}
