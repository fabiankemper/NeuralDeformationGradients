// =====================================================================================================================

#pragma once
#ifndef MESHCOMBINATION_H
#define MESHCOMBINATION_H

// =====================================================================================================================

#include <utility>
#include <volumetric_base/utility/utility.h>
#include "volumetric_base/state/volumetric_state.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(diffgeo)


// =====================================================================================================================



// =====================================================================================================================

class DifferentialGeometrySolver {
public:
    LayeredMesh mean;

    /**
     * Note: Have to call "initialize" before use!!
     * Is done so that it can both be initialized with cleaned and uncleaned Mesh
     */
    explicit DifferentialGeometrySolver(LayeredMesh mean) : mean(std::move(mean)) {initialize(mean);};
    DifferentialGeometrySolver() = default;
    void initialize(const pmp::SurfaceMesh &mean_skel, const pmp::SurfaceMesh &mean_musc,
                const pmp::SurfaceMesh &mean_skin);
    void initialize(VolumetricState *state);
    void initialize(const LayeredMesh & mesh);


    void setupTypeLaplacians();

    auto layerSelectorMatrix(volumetric_base::Layer type) const -> Eigen::SparseMatrix<double>;
    LayeredMesh solveFromDeformationDefgrads(const Eigen::MatrixXd &defgrads, const LayeredMesh &mesh);
    void solveFromDefgradsDirect(const Eigen::MatrixXd &defgrads, const LayeredMesh &mesh);


    //Precomputed Values
    Eigen::SparseMatrix<double> laplacian;
    Eigen::SparseMatrix<float> divergence;
    Eigen::SparseMatrix<double> muscleLaplacian;
    Eigen::SparseMatrix<double> skinLaplacian;
    PrecomputeSolver precomputedSolver;
    PrecomputeSolver solver;
    //only used to transform neural network results back into triangle mesh
    PrecomputeSolver precomputedNetworkSolver;
    Eigen::SparseMatrix<double> muscle_gradient;
    Eigen::SparseMatrix<double> skin_gradient;
    Eigen::SparseMatrix<double> gradient;
    Eigen::MatrixXf rhs;
    Eigen::MatrixXd results;
    Eigen::MatrixXd C;
    std::unordered_set<int> constrained_indices;

};

// =====================================================================================================================

NAMESPACE_END(mesh_combination)
NAMESPACE_END(volumetric_base)

// =====================================================================================================================

#endif //MESHCOMBINATION_H

// =====================================================================================================================
