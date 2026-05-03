#include "projective_skinning.h"

#include <pmp_character/DualQuaternion.h>

#ifdef WITH_IMGUI
#include "imgui.h"
#endif
#include "fast_rotation_fitting/cayley_rotation_fitting_util.hpp"
#include "fast_rotation_fitting/fit_rotation_avx.h"

#include "../base/basic_skinning.h"
#include "../base/volumetric_boundary_fixer.h"
#include "volumetric_base/diffgeo/prism_diffgeo.h"
#include "volumetric_base/state/skinning/skinning.h"

using namespace Eigen;
using namespace std;
using namespace volumetric_character;
using namespace volumetric_base;

NAMESPACE_BEGIN(projective_skinning)

/**
 *  Implementation of Projective Skinning without collision resolution
 */

/**
 * ASSUMPTIONS:
 *  mesh is cleaned and has been set up using
 * volumetric_skinning::SetupPropertiesForCleanedMesh Uses the very fast
 * rotation fitting of Zhang et al. (2021). Fast Updates for Least-Squares
 * Rotational Alignment
 */
inline auto getTriangleReferenceFrame(const volumetric_base::LayeredMesh &mesh,
                                      const pmp::Face f, Matrix3d &res)
    -> void {
  using namespace pmp;
  vector<Vector3d> points;
  for (Vertex v : mesh.vertices(f)) {
    points.emplace_back(mesh.position(v, Layer::SKEL));
  }

  Vector3d v1 = points[0] - points[1];
  Vector3d v2 = points[0] - points[2];
  res.col(0) = v1.normalized();
  res.col(1) = (v1.cross(v2)).normalized();
  res.col(2) = (res.col(0).cross(res.col(1))).normalized();
}

//======================================================================================================================

ProjectiveSkinner::ProjectiveSkinner(volumetric_base::LayeredMesh mesh) : undeformed_mesh(mesh) {
  cells = mesh.getPrisms();
  num_groups = ceil(static_cast<float>(cells.size()) / 8.f);
  rotations = vector<float>(num_groups * 8 * 9);
  deformation_grads = vector<float>(num_groups * 8 * 9);

  SparseMatrix<double> cell_volume_matrix =
      prism_diffgeo::volumeMatrixCells(mesh);
  gradient_matrix = prism_diffgeo::gradientMatrix(undeformed_mesh);
  divergence = gradient_matrix.transpose() * cell_volume_matrix;
  lefthandside = divergence * gradient_matrix;
}

//======================================================================================================================

void ProjectiveSkinner::update_skinning_with_dualquats(
    const volumetric_base::LayeredMesh &mesh, const VolumetricState &state) {
  using namespace std;
  using namespace pmp;
  using namespace Eigen;
  using dualquat = Eigen::Vector<double, 8>;
  auto rig = state.rig;
  auto blend_dual_quats =
      [&](const vector<dualquat> &dualquats,
          const vector<double> &weights) -> tuple<Matrix3d, Vector3d> {
    dualquat quatsum = dualquat::Zero();
    auto get_first_quat = [&](dualquat &quat) -> Vector4d {
      return {quat[0], quat[1], quat[2], quat[3]};
    };
    for (unsigned int i = 0; i < dualquats.size(); i++) {
      dualquat temp = dualquats[i];
      double dotprod = get_first_quat(quatsum).dot(get_first_quat(temp));
      dualquat add_val = temp * weights[i];
      quatsum += dotprod > 0 ? add_val : -add_val;
    }
    quatsum /= get_first_quat(quatsum).norm();
    auto transmatrix =
        static_cast<Matrix4d>(pmp_character::dualquat_to_mat4(quatsum));
    Matrix3d rota = transmatrix.block<3, 3>(0, 0);
    Vector3d translation = transmatrix.block<3, 1>(0, 3);
    return {rota, translation};
  };
  VertexProperty<ivec8> deps = mesh.get_vertex_property<ivec8>(VP_DEPENDS);
  VertexProperty<vec8> skinning_weights =
      mesh.get_vertex_property<vec8>(VP_SKINNING_WEIGHTS);
  rig.forward_kinematics();
  std::vector<mat4> _skinning_matrices;
  _skinning_matrices.resize(rig.joints_.size());
  for (unsigned int i = 0; i < rig.joints_.size(); ++i) {
    _skinning_matrices[i] = rig.joints_[i]->transformation_;
  }
#pragma omp parallel for
  for (int cell_idx = 0; cell_idx < static_cast<int>(cells.size()); cell_idx++) {
    const Prism &cell = cells[cell_idx];
    auto &[face, type] = cell;
    vector<dualquat> skinning_dualquats;
    vector<double> weights;
    auto [v1, v2, v3] = getFaceVertices(mesh, face);
    vector verts = {v1, v2, v3};
    for (Vertex v : verts) {
      for (int i = 0; i < 8; i++) {
        int depd = deps[v][i];
        double weight = skinning_weights[v][i];
        weights.emplace_back(weight);
        auto skinning_matrix = static_cast<Matrix4d>(_skinning_matrices[depd]);
        skinning_dualquats.emplace_back(
            static_cast<dualquat>(pmp_character::mat4_to_dualquat(
                static_cast<mat4>(skinning_matrix))));
      }
    }
    auto [r, t] = blend_dual_quats(skinning_dualquats, weights);
    int groupnum = static_cast<int>(cell_idx) / 8;
    int position_in_group = static_cast<int>(cell_idx) % 8;
    addMatrixIntoCayleyRepresentation(rotations, r, groupnum,
                                      position_in_group);
  }
  rotations_dirty = true;
}

//======================================================================================================================

void ProjectiveSkinner::do_global_step(const LayeredMesh &deformable_mesh) {

  if (rhs.rows() == 0) {
    rhs.resize(static_cast<int>(cells.size()) * 3, 3);
  }

  // Construct rhs
  OMP_PARALLEL_FOR()
  for (int i = 0; i < static_cast<int>(cells.size()); i++) {

    int groupnum = i / 8;
    int position_in_group = i % 8;
    Matrix3f rota =
        MatrixFromCayleyRepresentation(rotations, groupnum, position_in_group)
            .transpose();
    for (unsigned int j = 0; j < 3; j++) {
      for (unsigned int k = 0; k < 3; k++) {
        rhs(i * 3 + j, k) = static_cast<double>(rota(j, k));
      }
    }
  }

  Eigen::Matrix<double, -1, 3> righthandside = divergence * rhs;

  std::set<int> constrained_indices;
  C = deformable_mesh.toVertexMatrix();
  for (auto vv : deformable_mesh.getLayerVertices()) {
    auto [v, type] = vv;
    if (type == Layer::SKEL) {
      constrained_indices.emplace(LayeredMesh::Index(vv));
    }
  }

  results.noalias() = precompute_solver.solve(
      lefthandside, righthandside,
      [&](unsigned int i) {
        return constrained_indices.contains(static_cast<int>(i));
      },
      C);

  for (auto vv : deformable_mesh.getLayerVertices()) {
    deformable_mesh.position(vv) = results.row(LayeredMesh::Index(vv));
  }
}

//======================================================================================================================

void ProjectiveSkinner::do_local_step(const LayeredMesh &deformable_mesh) {
  C = deformable_mesh.toVertexMatrix();
  defgrads.noalias() = (gradient_matrix * C);

  OMP_PARALLEL_FOR()
  for (int i = 0; i < static_cast<int>(cells.size()); i++) {
    int groupnum = i / 8;
    int position_in_group = i % 8;
    Matrix3d defgrad = defgrads.block<3, 3>(i * 3, 0).transpose();
    addMatrixIntoCayleyRepresentation(deformation_grads, defgrad, groupnum,
                                      position_in_group);
  }
#ifdef HAVE_AVX2
  fit_rotation_avx(deformation_grads.data(), rotations.data(), num_groups);
#else
  fit_rotation_no_avx(deformation_grads.data(), rotations.data(), num_groups);
#endif
}

//======================================================================================================================

void ProjectiveSkinner::do_full_steps(const LayeredMesh &deformable_mesh,
                                      int num_steps) {
  for (int i = 0; i < num_steps; i++) {
    do_global_step(deformable_mesh);
    do_local_step(deformable_mesh);
  }
}

//======================================================================================================================
//                                               API Implementation
//======================================================================================================================

void ProjectiveSkinnerInterface::perform_skinning(
    volumetric_base::VolumetricState *state,
    volumetric_base::SkinnableMesh &skel, volumetric_base::SkinnableMesh &musc,
    volumetric_base::SkinnableMesh &skin, bool converge
) {
  volumetric_base::skinning::lbs(state, skel, volumetric_base::Layer::SKEL);
  if (converge)
    _animator->do_full_steps(_combined_copy, 100);

  pmp::SurfaceMesh new_skel = skel.bake_mesh();
  pmp::SurfaceMesh new_muscle = skel.bake_mesh();
  pmp::SurfaceMesh new_skin = skel.bake_mesh();

  prism_adapter::transfer_to_surface_mesh(new_skel, _combined_copy,
                                          volumetric_base::Layer::SKEL);
  prism_adapter::transfer_to_surface_mesh(new_muscle, _combined_copy,
                                          volumetric_base::Layer::MUSC);
  prism_adapter::transfer_to_surface_mesh(new_skin, _combined_copy,
                                          volumetric_base::Layer::SKIN);

  skel.skinning_from_baked_mesh(new_skel);
  musc.skinning_from_baked_mesh(new_muscle);
  skin.skinning_from_baked_mesh(new_skin);
}

//======================================================================================================================

void ProjectiveSkinnerInterface::updated_input_layers(volumetric_base::VolumetricState *state)
{
  if (!_diffgeo_solver) {
    std::cerr
        << "Called mesh combination without having instantiated mean meshes. "
        << std::endl;
    return;
  }
  rest_pose_mesh = state->get_layers();
  cleanMesh(rest_pose_mesh);
  delete _animator;
  _animator = new ProjectiveSkinner(rest_pose_mesh);
}

//======================================================================================================================

void ProjectiveSkinnerInterface::updated_mean_layers(volumetric_base::VolumetricState *state)
{
  const LayeredMesh mean = state->mean_mesh();

  delete _diffgeo_solver;
  _diffgeo_solver = new DiffgeoSolver(mean);
  _diffgeo_solver->initialize(state);
}

//======================================================================================================================

void ProjectiveSkinnerInterface::updated_pose(volumetric_base::VolumetricState *state)
{
  if (_animator == nullptr) {
    std::cerr
        << "Called mesh combination without having instantiated mean meshes. "
        << std::endl;
    return;
  }
  _combined_copy = rest_pose_mesh;
  volumetric_skinning::basic_skinning::setup_mesh_properties_for_cleaned_mesh(
      _combined_copy, state->get_skinning_parameters(Layer::SKEL));
  volumetric_skinning::basic_skinning::linear_blend_skin(_combined_copy, state);

  _animator->update_skinning_with_dualquats(_combined_copy, *state);
  _animator->do_global_step(_combined_copy);
}

//======================================================================================================================

void ProjectiveSkinnerInterface::imgui_update(volumetric_base::VolumetricState *state)
{
#ifdef WITH_IMGUI
  if (ImGui::Button("Projective Skinning Step")) {
    doStep();
    state->set_skinning_result(this->_combined_copy);
  };
#endif
}

//======================================================================================================================

void ProjectiveSkinnerInterface::doStep() const {
  _animator->do_full_steps(this->_combined_copy, 5);
}

NAMESPACE_END(projective_skinning)
