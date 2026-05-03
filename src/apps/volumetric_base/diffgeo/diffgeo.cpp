#include "diffgeo.h"

#include <set>
#include <unordered_set>

#include "pmp/stop_watch.h"
#include "volumetric_base/diffgeo/prism_diffgeo.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)

using namespace Eigen;
using namespace volumetric_base;

//======================================================================================================================
//                                       Utility functions.
//======================================================================================================================

NAMESPACE_BEGIN(diffgeo)
//======================================================================================================================
//                                                      Initialization
//======================================================================================================================

void DifferentialGeometrySolver::initialize(const pmp::SurfaceMesh &mean_skel,
                                const pmp::SurfaceMesh &mean_musc,
                                const pmp::SurfaceMesh &mean_skin) {
  LayeredMesh meanMesh(mean_skel, mean_musc, mean_skin);
  if (!meanMesh.is_cleaned()) {
    cleanMesh(meanMesh);
  }
  gradient = prism_diffgeo::gradientMatrix(meanMesh);
  SparseMatrix<double> volumeMatrix =
      prism_diffgeo::volumeMatrixCells(meanMesh);
  divergence = gradient.transpose() * volumeMatrix;

  laplacian = divergence.cast<double>() * gradient;
  gradient.makeCompressed();
  divergence.makeCompressed();
  laplacian.makeCompressed();
  mean = meanMesh;
  setupTypeLaplacians();

  muscle_gradient = layerSelectorMatrix(Layer::SKEL) * gradient;
  skin_gradient = layerSelectorMatrix(Layer::MUSC) * gradient;
}

//======================================================================================================================

void DifferentialGeometrySolver::initialize(VolumetricState *state) {
  auto [skel, musc, skin] = state->mean_mesh().toSurfaceMeshes();
  LayeredMesh meanMesh(skel, musc, skin);
  initialize(skel, musc, skin);
}

void DifferentialGeometrySolver::initialize(const LayeredMesh &mesh) {
  auto [m1, m2, m3] = mesh.toSurfaceMeshes();
  initialize(m1, m2, m3);
}

//======================================================================================================================

void DifferentialGeometrySolver::setupTypeLaplacians() {
  this->muscleLaplacian =
      divergence.cast<double>() * layerSelectorMatrix(Layer::SKEL) * gradient;
  this->skinLaplacian =
      divergence.cast<double>() * layerSelectorMatrix(Layer::MUSC) * gradient;
}

//======================================================================================================================

auto DifferentialGeometrySolver::layerSelectorMatrix(Layer type) const
    -> Eigen::SparseMatrix<double> {
  std::vector<Prism> cells = mean.getPrisms();
  SparseMatrix<double> mat(static_cast<int>(cells.size()) * 3,
                           static_cast<int>(cells.size()) * 3);
  std::vector<Triplet<double>> triplets;
  for (unsigned int i = 0; i < cells.size(); i++) {
    auto [_, celltype] = cells[i];
    if (celltype == type) {
      triplets.emplace_back(i * 3, i * 3, 1);
      triplets.emplace_back(i * 3 + 1, i * 3 + 1, 1);
      triplets.emplace_back(i * 3 + 2, i * 3 + 2, 1);
    }
  }
  mat.setFromTriplets(triplets.begin(), triplets.end());
  mat.makeCompressed();
  return mat;
}
//======================================================================================================================
//                                          Utility for DefGrad Construction
//======================================================================================================================

#include <chrono>

LayeredMesh
DifferentialGeometrySolver::solveFromDeformationDefgrads(const MatrixXd &defgrads,
                                             const LayeredMesh &mesh) {
  pmp::StopWatch watch;
  watch.start();
  LayeredMesh result = mesh;
  MatrixXd C = mesh.toVertexMatrix(); // Creates full vertex position matrix
  watch.stop();
  std::cout << "Init" << watch.elapsed() << "\n";
  watch.start();

  std::unordered_set<int> constrained_indices;
  for (LayerVertex v : mesh.getLayerVertices()) {
    auto [vertex, type] = v;
    if (type == Layer::SKEL)
      constrained_indices.emplace(static_cast<int>(LayeredMesh::Index(v)));
    if (mesh.is_boundary(vertex)) {
      constrained_indices.emplace(static_cast<int>(LayeredMesh::Index(v)));
    }
  }
  watch.stop();
  std::cout << "Constraints " << watch.elapsed() << "\n";
  watch.start();

  rhs.noalias() = divergence * defgrads.cast<float>();
  watch.stop();
  std::cout << "Div " << watch.elapsed() << "\n";
  watch.start();

  results.noalias() = precomputedNetworkSolver.solve(
      laplacian, rhs.cast<double>(),
      [&](unsigned int i) {
        return constrained_indices.contains(static_cast<int>(i));
      },
      C);
  watch.stop();
  std::cout << "Solve " << watch.elapsed() << "\n";
  watch.start();

#pragma omp parallel for schedule(static)
  for (int i = 0; i < mesh.n_layer_vertices(); ++i) {
    LayerVertex v{i / 3, getLayer(i % 3)};
    mesh.position(v) = results.row(i); // expose a by-index setter
  }
  watch.stop();
  std::cout << "Write " << watch.elapsed() << "\n";
  watch.start();

  return result;
}

void DifferentialGeometrySolver::solveFromDefgradsDirect(
    const Eigen::MatrixXd &defgrads, const LayeredMesh &mesh) {
  C.noalias() = mesh.toVertexMatrix(); // Creates full vertex position matrix
  if (constrained_indices.empty()) {
    for (const LayerVertex &v : mesh.getLayerVertices()) {
      const auto &[vertex, type] = v;
      if (type == Layer::SKEL) {
        constrained_indices.emplace(static_cast<int>(LayeredMesh::Index(v)));
      }
      if (mesh.is_boundary(vertex)) {
        constrained_indices.emplace(static_cast<int>(LayeredMesh::Index(v)));
      }
    }
  }

  rhs.noalias() = divergence * defgrads.cast<float>();
  results.noalias() = precomputedNetworkSolver.solve(
      laplacian, rhs.cast<double>(),
      [&](const unsigned int i) {
        return constrained_indices.contains(static_cast<int>(i));
      },
      C);

#pragma omp parallel for schedule(static) default(none) shared(mesh)
  for (int i = 0; i < mesh.n_layer_vertices(); ++i) {
    const LayerVertex v{i / 3, getLayer(i % 3)};
    mesh.position(v) = results.row(i);
  }
}

// =====================================================================================================================

NAMESPACE_END(mesh_combination)

// =====================================================================================================================

NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================
