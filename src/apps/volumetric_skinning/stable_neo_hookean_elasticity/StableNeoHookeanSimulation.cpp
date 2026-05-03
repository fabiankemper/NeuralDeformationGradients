#include "StableNeoHookeanSimulation.h"

#include "../base/basic_skinning.h"
#include "../base/io.hpp"
#include "neo_hookean_derivatives.h"
#include "volumetric_base/diffgeo/prism_diffgeo.h"
#include "volumetric_skinning/thin_shell/bending_energy.h"

volumetric_skinning::neo_hookean::StableNeoHookeanSkinning::
    StableNeoHookeanSkinning(volumetric_base::LayeredMesh &mesh) {
  int i = 0;
  // Contains -1 if vertex is constrained or i if its the i-th unconstrained
  // vertex
  constrained_mapping = std::vector<int>(mesh.n_layer_vertices());
  for (const auto &[v, layer] : mesh.getLayerVertices()) {
    constrained_mapping[volumetric_base::LayeredMesh::Index(v, layer)] =
        (layer == volumetric_base::Layer::SKEL || mesh.is_boundary(v)) ? -1
                                                                       : i++;
  }
  bending_energy.setup(mesh, constrained_mapping);
  stretching_energy.setup(mesh, constrained_mapping);
  neo_hookean_energy.setup(mesh, constrained_mapping);
  neo_hookean_energy_quadrature.setup(mesh, constrained_mapping);
  neo_hookean_energy_tets.setup(mesh, constrained_mapping);

  undeformed_mesh = mesh;
}

double
volumetric_skinning::neo_hookean::StableNeoHookeanSkinning::compute_energy(
    const volumetric_base::LayeredMesh &layered_mesh) {
  double energy =
      bending_energy.energy(layered_mesh) * skin_bending_energy_weight +
      stretching_energy.energy(layered_mesh) * skin_stretching_energy_weight;
  if (discretization == Discretization::Prism) {
    energy += neo_hookean_energy.energy(layered_mesh);
  } else if (discretization == Discretization::Quadrature) {
    energy += neo_hookean_energy_quadrature.energy(layered_mesh);
  } else if (discretization == Discretization::Tets) {
    energy += neo_hookean_energy_tets.energy(layered_mesh);
  }
  return energy;
}

Eigen::VectorXd
volumetric_skinning::neo_hookean::StableNeoHookeanSkinning::elastic_gradient(
    const volumetric_base::LayeredMesh &deformed_mesh) {
  if (discretization == Discretization::Prism) {
    return neo_hookean_energy.gradient(deformed_mesh);
  }
  if (discretization == Discretization::Quadrature) {
    return neo_hookean_energy_quadrature.gradient(deformed_mesh);
  }
  return neo_hookean_energy_tets.gradient(deformed_mesh);
}

Eigen::SparseMatrix<double>
volumetric_skinning::neo_hookean::StableNeoHookeanSkinning::elastic_hessian(
    const volumetric_base::LayeredMesh &deformed_mesh, bool &need_update) {
  if (discretization == Discretization::Prism) {
    return neo_hookean_energy.hessian(deformed_mesh, need_update);
  }
  if (discretization == Discretization::Quadrature) {
    return neo_hookean_energy_quadrature.hessian(deformed_mesh, need_update);
  }
  return neo_hookean_energy_tets.hessian(deformed_mesh, need_update);
}

std::tuple<double, bool>
volumetric_skinning::neo_hookean::StableNeoHookeanSkinning::do_step(
    const volumetric_base::LayeredMesh &deformed_mesh) {
  double start_energy = compute_energy(deformed_mesh);

  const Eigen::VectorXd gradient =
      elastic_gradient(deformed_mesh) +
      bending_energy.gradient(deformed_mesh) * skin_bending_energy_weight +
      stretching_energy.gradient(deformed_mesh) * skin_stretching_energy_weight;

  const Eigen::SparseMatrix<double> H_elastic =
      elastic_hessian(deformed_mesh, patternAnalyzed);
  const Eigen::SparseMatrix<double> H_bend =
      bending_energy.hessian(deformed_mesh, patternAnalyzed);
  const Eigen::SparseMatrix<double> H_stretch =
      stretching_energy.hessian(deformed_mesh, patternAnalyzed);
  ;

  const Eigen::SparseMatrix<double> hessian =
      H_elastic + skin_bending_energy_weight * H_bend +
      skin_stretching_energy_weight * H_stretch;

  if (!patternAnalyzed) {
    solver.compute(hessian);
    patternAnalyzed = true;
  } else {
    solver.factorize(hessian);
  }

  const Eigen::VectorXd update = -solver.solve(gradient);

  const Eigen::VectorXd posis = deformed_mesh.toVertexVector();

  for (int k = 0; k < num_linesearch_steps; k++) {
    double factor = pow(0.5, k);
    Eigen::VectorXd test = factor * update;
    deformed_mesh.apply_constrained_update(test, constrained_mapping);
    double energy = compute_energy(deformed_mesh);
    if (energy < start_energy) {
      return {energy, false};
    }
    deformed_mesh.apply_constrained_update(-test, constrained_mapping);
  }

  return {start_energy, true};
}

std::vector<double>
volumetric_skinning::neo_hookean::StableNeoHookeanSkinning::do_skinning_steps(
    const volumetric_base::LayeredMesh &deformed_mesh, const int iterations) {
  std::vector<double> energies;
  for (int i = 0; i < iterations; i++) {
    auto [energy, converged] = do_step(deformed_mesh);
    energies.emplace_back(energy);
    if (converged) {
      return energies;
    }
  }
  return energies;
}

volumetric_base::LayeredMesh
volumetric_skinning::neo_hookean::StableNeoHookeanSkinning::animate(
    const volumetric_base::VolumetricState &state) {
  volumetric_base::LayeredMesh deformable_mesh = undeformed_mesh;
  basic_skinning::linear_blend_skin(deformable_mesh, &state);
  do_skinning_steps(deformable_mesh, 30);
  return deformable_mesh;
}

void volumetric_skinning::neo_hookean::StableNeoHookeanSkinning::animate(
    const volumetric_base::LayeredMesh &deformable_mesh,
    [[maybe_unused]] const volumetric_base::VolumetricState &state) {
  do_skinning_steps(deformable_mesh, 30);
}
