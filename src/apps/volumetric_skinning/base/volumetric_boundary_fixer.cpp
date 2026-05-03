// =====================================================================================================================
#include "volumetric_base/aliases.h"
#include "volumetric_base/layered_mesh.h"
#include "volumetric_base/state/volumetric_state.h"
#include "volumetric_base/utility/constants.h"
#include "volumetric_boundary_fixer.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_skinning)

/**
 *  This class is used to reattach extremities to a body.
 */

// =====================================================================================================================

using namespace pmp;
using namespace Eigen;

// =====================================================================================================================

using LayeredMesh = volumetric_base::LayeredMesh;
using Selection = volumetric_base::Selection;
using VolumetricState = volumetric_base::VolumetricState;

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

void computeBestAffineTransformation(const std::vector<Eigen::Vector3d> &source,
                                     const std::vector<Eigen::Vector3d> &target,
                                     Eigen::Matrix3d &rotation,
                                     Eigen::Vector3d &translation) {
  const int num_points = static_cast<int>(source.size());
  Vector3d centroid_source = Eigen::Vector3d::Zero();
  Vector3d centroid_target = Eigen::Vector3d::Zero();
  for (int i = 0; i < num_points; ++i) {
    centroid_source += source[i];
    centroid_target += target[i];
  }
  centroid_source /= num_points;
  centroid_target /= num_points;
  // Compute the centered vectors
  MatrixXd centered_source(3, num_points);
  MatrixXd centered_target(3, num_points);
  for (int i = 0; i < num_points; ++i) {
    centered_source.col(i) = source[i] - centroid_source;
    centered_target.col(i) = target[i] - centroid_target;
  }
  Matrix3d covariance_matrix = centered_source * centered_target.transpose();
  JacobiSVD svd(covariance_matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Matrix3d U = svd.matrixU();
  Matrix3d V = svd.matrixV();
  rotation = V * U.transpose();
  if (rotation.determinant() < 0) {
    V.col(2) *= -1;
    rotation = V * U.transpose();
  }
  translation = centroid_target - rotation * centroid_source;
}

// ---------------------------------------------------------------------------------------------------------------------

VolumetricBoundaryFixer::VolumetricBoundaryFixer(LayeredMesh &mesh) {
  border_regions = find_border_regions(mesh);
}

// ---------------------------------------------------------------------------------------------------------------------

void mark_as_dirty(VolumetricState *state, SurfaceMesh &mesh) {
  auto dirty = mesh.get_vertex_property<bool>("v:dirty");

  const Selection &selection = state->collapsed_vertices;
  for (const auto idx : selection) {
    const Vertex vertex(idx);
    dirty[vertex] = true;
  }
}

// ---------------------------------------------------------------------------------------------------------------------

auto set_all_neighboring_dirty(SurfaceMesh &mesh, Vertex v,
                               const Matrix3d &rotation,
                               const Vector3d &translation) -> void {
  auto dirty = mesh.get_vertex_property<bool>("v:dirty");

  if (!dirty[v]) {
    return;
  }
  Vector3d newpos =
      rotation * static_cast<Vector3d>(mesh.position(v)) + translation;
  mesh.position(v) = newpos;
  dirty[v] = false;
  for (Vertex vv : mesh.vertices(v)) {
    set_all_neighboring_dirty(mesh, vv, rotation, translation);
  }
}

// ---------------------------------------------------------------------------------------------------------------------

auto set_all_neighboring_dirty(SurfaceMesh &mesh, SurfaceMesh &target, Vertex v,
                               const Matrix3d &rotation,
                               const Vector3d &translation) -> void {
  auto dirty = mesh.get_vertex_property<bool>("v:dirty");

  if (!dirty[v]) {
    return;
  }
  Vector3d newpos =
      rotation * static_cast<Vector3d>(target.position(v)) + translation;
  mesh.position(v) = newpos;
  dirty[v] = false;
  for (Vertex vv : mesh.vertices(v)) {
    set_all_neighboring_dirty(mesh, target, vv, rotation, translation);
  }
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricBoundaryFixer::reattach_body_parts(
    volumetric_base::VolumetricState *state, pmp::SurfaceMesh &origin,
    pmp::SurfaceMesh &target) -> void {
  if (target.has_vertex_property("v:dirty")) {
    VertexProperty d = target.get_vertex_property<bool>("v:dirty");
    target.remove_vertex_property(d);
  }

  target.add_vertex_property<bool>("v:dirty", false);
  mark_as_dirty(state, target);

  for (unsigned int idx = 0; idx < border_regions.size(); idx++) {
    std::vector<Vector3d> origins;
    std::vector<Vector3d> targets;
    for (const int index : border_regions[idx]) {
      const Vertex vertex(index);
      origins.emplace_back(origin.position(vertex));
      targets.emplace_back(target.position(vertex));
    }
    Matrix3d rotation;
    Vector3d translation;
    computeBestAffineTransformation(origins, targets, rotation, translation);
    for (int vertex_index : border_regions[idx]) {
      Vertex v(vertex_index);
      for (Vertex vv : target.vertices(v)) {
        set_all_neighboring_dirty(target, origin, vv, rotation, translation);
      }
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------------

void VolumetricBoundaryFixer::transfer_fixes(const VolumetricState *state,
                                             const SurfaceMesh &surface_mesh,
                                             SurfaceMesh &new_muscle,
                                             SurfaceMesh &new_skel) {
  const Selection &selection = state->collapsed_vertices;

  for (const pmp::IndexType idx : selection) {
    const Vertex vertex(idx);
    const Vector3d pos = surface_mesh.position(vertex);
    new_muscle.position(vertex) = pos;
    new_skel.position(vertex) = pos;
  }
}

// ---------------------------------------------------------------------------------------------------------------------

SurfaceMesh VolumetricBoundaryFixer::linear_blend_skin(
    SurfaceMesh mesh, volumetric_base::Skeleton &skeleton,
    volumetric_base::SurfaceMesh &propsource) {
  skeleton.forward_kinematics();
  std::vector<pmp::mat4> _skinning_matrices;

  _skinning_matrices.resize(skeleton.joints_.size());
  for (unsigned int i = 0; i < skeleton.joints_.size(); ++i) {
    _skinning_matrices[i] = skeleton.joints_[i]->transformation_;
  }

  VertexProperty<pmp_character::ivec8> dependencies =
      propsource.vertex_property<pmp_character::ivec8>(
          volumetric_base::VP_DEPENDS);
  VertexProperty<pmp_character::vec8> weights =
      propsource.vertex_property<pmp_character::vec8>(
          volumetric_base::VP_SKINNING_WEIGHTS);
  for (Vertex v : mesh.vertices()) {
    Vector4d new_position = Vector4d::Zero();
    pmp_character::ivec8 deps = dependencies[v];
    pmp_character::vec8 wghts = weights[v];
    Vector4d position;
    position.block<3, 1>(0, 0) = static_cast<Vector3d>(mesh.position(v));
    position[3] = 1;
    for (int i = 0; i < 8; i++) {
      auto transform = static_cast<Matrix4d>(_skinning_matrices[deps[i]]);
      new_position += wghts[i] * transform * position;
    }
    mesh.position(v) = new_position.block<3, 1>(0, 0);
  }
  return mesh;
}

// ---------------------------------------------------------------------------------------------------------------------

auto compute_boundary_ring(Vertex v, std::vector<int> &vec, LayeredMesh &mesh)
    -> void {
  VertexProperty is_processed = mesh.get_vertex_property<bool>("v:processed");
  pmp::VertexProperty clean_indices =
      mesh.vertex_property<pmp::IndexType>(volumetric_base::VP_OLD_INDICES);
  if (!mesh.is_boundary(v)) {
    return;
  }
  if (is_processed[v]) {
    return;
  }
  vec.emplace_back(clean_indices[v]);
  is_processed[v] = true;
  for (Vertex vv : mesh.vertices(v)) {
    compute_boundary_ring(vv, vec, mesh);
  }
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricBoundaryFixer::find_border_regions(LayeredMesh &mesh)
    -> std::vector<std::vector<int>> {
  using namespace std;
  vector<vector<int>> regions;
  VertexProperty is_processed =
      mesh.add_vertex_property<bool>("v:processed", false);

  for (Vertex v : mesh.vertices()) {
    if (is_processed[v]) {
      continue;
    }
    if (mesh.is_boundary(v)) {
      std::vector<int> boundary_ring;
      compute_boundary_ring(v, boundary_ring, mesh);
      regions.emplace_back(boundary_ring);
    }
  }
  mesh.remove_vertex_property(is_processed);
  return regions;
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================
