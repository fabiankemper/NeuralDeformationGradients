#include "basic_skinning.h"
#include "pmp_character/DualQuaternion.h"
#include "volanim_utils/defs.h"
#include "volumetric_base/utility/constants.h"
#include "volumetric_base/utility/mesh_util.h"
#include "volumetric_base/utility/precompute_solver.h"
#include <cfloat>

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_skinning)
NAMESPACE_BEGIN(basic_skinning)

// =====================================================================================================================

using vec8 = volumetric_base::vec8;
using ivec8 = volumetric_base::ivec8;
using namespace volumetric_base;
// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

auto dual_quat_skin_skeleton(const volumetric_base::LayeredMesh &mesh,
    const volumetric_base::VolumetricState *const state)
    -> void {
  using namespace std;
  using namespace pmp;
  using namespace Eigen;
  using dualquat = Eigen::Vector<double, 8>;

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
  VertexProperty<ivec8> deps =
      mesh.get_vertex_property<ivec8>(volumetric_base::VP_DEPENDS);
  VertexProperty<vec8> skinning_weights =
      mesh.get_vertex_property<vec8>(volumetric_base::VP_SKINNING_WEIGHTS);
  state->rig.forward_kinematics();
  std::vector<mat4> _skinning_matrices;
  _skinning_matrices.resize(state->rig.joints_.size());
  for (unsigned int i = 0; i < state->rig.joints_.size(); ++i) {
    _skinning_matrices[i] = state->rig.joints_[i]->transformation_;
  }
  for (unsigned int v_idx = 0; v_idx < mesh.n_vertices(); v_idx++) {
    Vertex v(v_idx);
    vector<dualquat> skinning_dualquats;
    vector<double> weights;
    for (int i = 0; i < 8; i++) {
      int depd = deps[v][i];
      double weight = skinning_weights[v][i];
      weights.emplace_back(weight);
      auto skinning_matrix = static_cast<Matrix4d>(_skinning_matrices[depd]);
      skinning_dualquats.emplace_back(static_cast<dualquat>(
          pmp_character::mat4_to_dualquat(static_cast<mat4>(skinning_matrix))));
    }

    auto [r, t] = blend_dual_quats(skinning_dualquats, weights);
    mesh.position(v, Layer::SKEL) = r * mesh.position(v, Layer::SKEL) + t;
    mesh.position(v, Layer::MUSC) = r * mesh.position(v, Layer::MUSC) + t;
    mesh.position(v, Layer::SKIN) = r * mesh.position(v, Layer::SKIN) + t;
  }
}

Eigen::MatrixXd get_dqs_rotations(const LayeredMesh &mesh,
                                  const VolumetricState &state) {
  using namespace std;
  using namespace pmp;
  using namespace Eigen;
  using dualquat = Eigen::Vector<double, 8>;

  Eigen::MatrixXd rotations(mesh.n_faces() * 3, 3);

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
  VertexProperty<ivec8> deps =
      mesh.get_vertex_property<ivec8>(volumetric_base::VP_DEPENDS);
  VertexProperty<vec8> skinning_weights =
      mesh.get_vertex_property<vec8>(volumetric_base::VP_SKINNING_WEIGHTS);
  rig.forward_kinematics();
  std::vector<mat4> _skinning_matrices;
  _skinning_matrices.resize(rig.joints_.size());
  for (unsigned int i = 0; i < rig.joints_.size(); ++i) {
    _skinning_matrices[i] = rig.joints_[i]->transformation_;
  }
  for (unsigned int i = 0; i < mesh.n_faces(); i++) {
    Face face(i);
    vector<dualquat> skinning_dualquats;
    vector<double> weights;
    auto [v1, v2, v3] = getFaceVertices(mesh, face);
    vector verts = {v1, v2, v3};
    for (Vertex v : verts) {
      for (int idx = 0; idx < 8; idx++) {
        int depd = deps[v][idx];
        double weight = skinning_weights[v][idx];
        weights.emplace_back(weight);
        auto skinning_matrix = static_cast<Matrix4d>(_skinning_matrices[depd]);
        skinning_dualquats.emplace_back(
            static_cast<dualquat>(pmp_character::mat4_to_dualquat(
                static_cast<mat4>(skinning_matrix))));
      }
    }
    auto [r, t] = blend_dual_quats(skinning_dualquats, weights);
    rotations.block<3, 3>(face.idx() * 3, 0) = r;
  }
  return rotations;
}

// ---------------------------------------------------------------------------------------------------------------------

void linear_blend_skin_skeleton(LayeredMesh &mesh,
                                const VolumetricState *const state) {
  using namespace pmp;
  using namespace Eigen;
  using namespace std;

  state->rig.forward_kinematics();
  std::vector<mat4> _skinning_matrices;

  _skinning_matrices.resize(state->rig.joints_.size());
  for (unsigned int i = 0; i < state->rig.joints_.size(); ++i) {
    _skinning_matrices[i] = state->rig.joints_[i]->transformation_;
  }
  auto dependencies =
      mesh.vertex_property<pmp_character::ivec8>(volumetric_base::VP_DEPENDS);
  auto weights = mesh.vertex_property<pmp_character::vec8>(
      volumetric_base::VP_SKINNING_WEIGHTS);

  OMP_PARALLEL_FOR()
  for (long v_idx = 0; v_idx < static_cast<long>(mesh.n_vertices()); v_idx++) {
    const Vertex v(v_idx);
    Vector3d newSkelPosition = Vector3d::Zero();

    Vector4d affine_old_position_skel;
    affine_old_position_skel.block<3, 1>(0, 0) =
        static_cast<Vector3d>(mesh.position(v, Layer::SKEL));
    affine_old_position_skel[3] = 1;

    for (int param_idx = 0; param_idx < 8; param_idx++) {
      int depd = dependencies[v][param_idx];
      float weight = weights[v][param_idx];
      auto skinning_matrix = static_cast<Matrix4d>(_skinning_matrices[depd]);
      newSkelPosition +=
          weight *
          (skinning_matrix * affine_old_position_skel).block<3, 1>(0, 0);
    }
    mesh.position(v, Layer::SKEL) = newSkelPosition;
  }
}

//---------------------------------------------------------------------------------------------------------------------

auto linear_blend_skinning_matrices(LayeredMesh &deformable_mesh,
                                    const VolumetricState *state)
    -> std::vector<Eigen::Matrix4d> {
  using namespace pmp;
  using namespace Eigen;
  using namespace std;

  std::vector<Eigen::Matrix4d> result(deformable_mesh.n_layer_vertices());
  state->rig.forward_kinematics();
  std::vector<mat4> _skinning_matrices;

  _skinning_matrices.resize(state->rig.joints_.size());
  for (unsigned int i = 0; i < state->rig.joints_.size(); ++i) {
    _skinning_matrices[i] = state->rig.joints_[i]->transformation_;
  }
  auto dependencies = deformable_mesh.vertex_property<pmp_character::ivec8>(
      volumetric_base::VP_DEPENDS);
  auto weights = deformable_mesh.vertex_property<pmp_character::vec8>(
      volumetric_base::VP_SKINNING_WEIGHTS);
  OMP_PARALLEL_FOR()
  for (long i = 0; i < static_cast<long>(deformable_mesh.n_vertices()); i++) {
    Vertex v(i);
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Zero();
    for (int j = 0; j < 8; j++) {
      int depd = dependencies[v][j];
      float weight = weights[v][j];
      auto skinning_matrix = static_cast<Matrix4d>(_skinning_matrices[depd]);
      matrix += weight * (skinning_matrix);
    }
    result[deformable_mesh.Index(v, Layer::SKEL)] = matrix;
    result[deformable_mesh.Index(v, Layer::MUSC)] = matrix;
    result[deformable_mesh.Index(v, Layer::SKIN)] = matrix;
  }

  return result;
}

//----------------------------------------------------------------------------------------------------------------------

auto inverse_linear_blend_skin(LayeredMesh &mesh, VolumetricState const *state)
    -> void {
  using namespace pmp;
  using namespace Eigen;
  using namespace std;
  state->rig.forward_kinematics();
  std::vector<mat4> _skinning_matrices;

  _skinning_matrices.resize(state->rig.joints_.size());
  for (unsigned int i = 0; i < state->rig.joints_.size(); ++i) {
    _skinning_matrices[i] = state->rig.joints_[i]->transformation_;
  }
  auto dependencies =
      mesh.vertex_property<pmp_character::ivec8>(volumetric_base::VP_DEPENDS);
  auto weights = mesh.vertex_property<pmp_character::vec8>(
      volumetric_base::VP_SKINNING_WEIGHTS);
  for (unsigned int v_idx = 0; v_idx < mesh.n_vertices(); v_idx++) {
    Vertex v(v_idx);
    Vector4d affine_old_position_skel, affine_old_position_musc,
        affine_old_position_skin;
    affine_old_position_skel.block<3, 1>(0, 0) = mesh.position(v, Layer::SKEL);
    affine_old_position_skel[3] = 1;
    affine_old_position_musc.block<3, 1>(0, 0) = mesh.position(v, Layer::MUSC);
    affine_old_position_musc[3] = 1;
    affine_old_position_skin.block<3, 1>(0, 0) = mesh.position(v, Layer::SKIN);
    affine_old_position_skin[3] = 1;
    Matrix4d vertex_skinning_matrix = Eigen::Matrix4d::Zero();
    for (int param_idx = 0; param_idx < 8; param_idx++) {
      int depd = dependencies[v][param_idx];
      float weight = weights[v][param_idx];
      auto skinning_matrix = static_cast<Matrix4d>(_skinning_matrices[depd]);
      vertex_skinning_matrix += weight * skinning_matrix;
    }
    vertex_skinning_matrix = vertex_skinning_matrix.inverse();
    mesh.position(v, Layer::SKEL) =
        (vertex_skinning_matrix * affine_old_position_skel).block<3, 1>(0, 0);
    mesh.position(v, Layer::MUSC) =
        (vertex_skinning_matrix * affine_old_position_musc).block<3, 1>(0, 0);
    mesh.position(v, Layer::SKIN) =
        (vertex_skinning_matrix * affine_old_position_skin).block<3, 1>(0, 0);
  }
}

// ---------------------------------------------------------------------------------------------------------------------

auto inverse_linear_blend_skin_safe(LayeredMesh &mesh,
                                    VolumetricState const *state) -> double {
  using namespace pmp;
  using namespace Eigen;
  using namespace std;
  state->rig.forward_kinematics();
  std::vector<mat4> _skinning_matrices;

  _skinning_matrices.resize(state->rig.joints_.size());
  for (unsigned int i = 0; i < state->rig.joints_.size(); ++i) {
    _skinning_matrices[i] = state->rig.joints_[i]->transformation_;
  }
  auto dependencies =
      mesh.vertex_property<pmp_character::ivec8>(volumetric_base::VP_DEPENDS);
  auto weights = mesh.vertex_property<pmp_character::vec8>(
      volumetric_base::VP_SKINNING_WEIGHTS);
  auto min_det = DBL_MAX;
  for (unsigned int v_idx = 0; v_idx < mesh.n_vertices(); v_idx++) {
    Vertex v(v_idx);
    Vector4d affine_old_position_skel, affine_old_position_musc,
        affine_old_position_skin;
    affine_old_position_skel.block<3, 1>(0, 0) = mesh.position(v, Layer::SKEL);
    affine_old_position_skel[3] = 1;
    affine_old_position_musc.block<3, 1>(0, 0) = mesh.position(v, Layer::MUSC);
    affine_old_position_musc[3] = 1;
    affine_old_position_skin.block<3, 1>(0, 0) = mesh.position(v, Layer::SKIN);
    affine_old_position_skin[3] = 1;
    Matrix4d vertex_skinning_matrix = Eigen::Matrix4d::Zero();
    for (int param_idx = 0; param_idx < 8; param_idx++) {
      int depd = dependencies[v][param_idx];
      float weight = weights[v][param_idx];
      auto skinning_matrix = static_cast<Matrix4d>(_skinning_matrices[depd]);
      vertex_skinning_matrix += weight * skinning_matrix;
    }
    min_det = min(min_det, vertex_skinning_matrix.determinant());

    vertex_skinning_matrix = vertex_skinning_matrix.inverse();
    mesh.position(v, Layer::SKEL) =
        (vertex_skinning_matrix * affine_old_position_skel).block<3, 1>(0, 0);
    mesh.position(v, Layer::MUSC) =
        (vertex_skinning_matrix * affine_old_position_musc).block<3, 1>(0, 0);
    mesh.position(v, Layer::SKIN) =
        (vertex_skinning_matrix * affine_old_position_skin).block<3, 1>(0, 0);
  }
  return min_det;
}

// ---------------------------------------------------------------------------------------------------------------------

void linear_blend_skin(LayeredMesh &mesh, VolumetricState const *state) {
  using namespace pmp;
  using namespace Eigen;
  using namespace std;

  state->rig.forward_kinematics();
  std::vector<mat4> _skinning_matrices;

  _skinning_matrices.resize(state->rig.joints_.size());
  for (unsigned int i = 0; i < state->rig.joints_.size(); ++i) {
    _skinning_matrices[i] = state->rig.joints_[i]->transformation_;
  }
  auto dependencies =
      mesh.vertex_property<pmp_character::ivec8>(volumetric_base::VP_DEPENDS);
  auto weights = mesh.vertex_property<pmp_character::vec8>(
      volumetric_base::VP_SKINNING_WEIGHTS);

  OMP_PARALLEL_FOR()
  for (int v_idx = 0; v_idx < static_cast<int>(mesh.n_vertices()); v_idx++) {
    Vertex v(v_idx);
    Vector3d newSkelPosition = Vector3d::Zero();
    Vector3d newMuscPosition = Vector3d::Zero();
    Vector3d newSkinPosition = Vector3d::Zero();

    Vector4d affine_old_position_skel, affine_old_position_musc,
        affine_old_position_skin;
    affine_old_position_skel.block<3, 1>(0, 0) =
        static_cast<Vector3d>(mesh.position(v, Layer::SKEL));
    affine_old_position_skel[3] = 1;
    affine_old_position_musc.block<3, 1>(0, 0) =
        static_cast<Vector3d>(mesh.position(v, Layer::MUSC));
    affine_old_position_musc[3] = 1;
    affine_old_position_skin.block<3, 1>(0, 0) =
        static_cast<Vector3d>(mesh.position(v, Layer::SKIN));
    affine_old_position_skin[3] = 1;
    for (int param_idx = 0; param_idx < 8; param_idx++) {
      int depd = dependencies[v][param_idx];
      float weight = weights[v][param_idx];
      auto skinning_matrix = static_cast<Matrix4d>(_skinning_matrices[depd]);
      newSkelPosition +=
          weight *
          (skinning_matrix * affine_old_position_skel).block<3, 1>(0, 0);
      newMuscPosition +=
          weight *
          (skinning_matrix * affine_old_position_musc).block<3, 1>(0, 0);
      newSkinPosition +=
          weight *
          (skinning_matrix * affine_old_position_skin).block<3, 1>(0, 0);
    }
    mesh.position(v, Layer::SKEL) = newSkelPosition;
    mesh.position(v, Layer::MUSC) = newMuscPosition;
    mesh.position(v, Layer::SKIN) = newSkinPosition;
  }
}

// ---------------------------------------------------------------------------------------------------------------------

void setup_mesh_properties_for_cleaned_mesh(
    LayeredMesh &mesh, const pmp::SurfaceMesh &propSource) {
  using namespace pmp;
  using namespace std;
  auto dependencies = propSource.get_vertex_property<pmp_character::ivec8>(
      volumetric_base::VP_DEPENDS);
  auto dependencies_new =
      mesh.vertex_property<pmp_character::ivec8>(volumetric_base::VP_DEPENDS);
  auto weights = propSource.get_vertex_property<pmp_character::vec8>(
      volumetric_base::VP_SKINNING_WEIGHTS);
  auto weights_new = mesh.vertex_property<pmp_character::vec8>(
      volumetric_base::VP_SKINNING_WEIGHTS);
  VertexProperty clean_indices =
      mesh.vertex_property<pmp::IndexType>(volumetric_base::VP_OLD_INDICES);
  for (Vertex v : mesh.vertices()) {
    Vertex vv(clean_indices[v]);
    dependencies_new[v] = dependencies[vv];
    weights_new[v] = weights[vv];
  }
}

//----------------------------------------------------------------------------------------------------------------------
void setup_mesh_properties_for_cleaned_mesh(
    pmp::SurfaceMesh &mesh, const volumetric_base::SkinningParameters &params) {
  using namespace pmp;
  using namespace std;
  auto dependencies_new =
      mesh.vertex_property<pmp_character::ivec8>(volumetric_base::VP_DEPENDS);
  auto weights_new = mesh.vertex_property<pmp_character::vec8>(
      volumetric_base::VP_SKINNING_WEIGHTS);
  VertexProperty clean_indices =
      mesh.vertex_property<pmp::IndexType>(volumetric_base::VP_OLD_INDICES);

  for (Vertex v : mesh.vertices()) {
    const auto &[v_depends, v_weights] = params[clean_indices[v]];
    dependencies_new[v] = v_depends;
    weights_new[v] = v_weights;
  }
}

void setup_mesh_properties_for_uncleaned_mesh(
    pmp::SurfaceMesh &mesh, const volumetric_base::SkinningParameters &params) {
  using namespace pmp;
  using namespace std;
  auto dependencies_new =
      mesh.vertex_property<pmp_character::ivec8>(volumetric_base::VP_DEPENDS);
  auto weights_new = mesh.vertex_property<pmp_character::vec8>(
      volumetric_base::VP_SKINNING_WEIGHTS);

  for (Vertex v : mesh.vertices()) {
    const auto &[v_depends, v_weights] = params[v.idx()];
    dependencies_new[v] = v_depends;
    weights_new[v] = v_weights;
  }
}

// ---------------------------------------------------------------------------------------------------------------------

void setup_mesh_properties_for_uncleaned_mesh(
    LayeredMesh &mesh, const pmp::SurfaceMesh &propSource) {
  using namespace pmp;
  using namespace std;
  auto dependencies = propSource.get_vertex_property<pmp_character::ivec8>(
      volumetric_base::VP_DEPENDS);
  auto dependencies_new =
      mesh.vertex_property<pmp_character::ivec8>(volumetric_base::VP_DEPENDS);
  auto weights = propSource.get_vertex_property<pmp_character::vec8>(
      volumetric_base::VP_SKINNING_WEIGHTS);
  auto weights_new = mesh.vertex_property<pmp_character::vec8>(
      volumetric_base::VP_SKINNING_WEIGHTS);

  for (Vertex v : mesh.vertices()) {
    dependencies_new[v] = dependencies[v];
    weights_new[v] = weights[v];
  }
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(basic_skinning)

NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================
