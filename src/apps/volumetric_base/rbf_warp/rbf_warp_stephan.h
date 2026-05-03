// =====================================================================================================================
// Created by fabian on 4/9/25.
// mail@fabiankemper.de
// =====================================================================================================================

#pragma once
#ifndef VOLUMETRIC_BASE_RBF_WARP_STEPHAN_H
#define VOLUMETRIC_BASE_RBF_WARP_STEPHAN_H

// =====================================================================================================================

#include <Eigen/Eigen>
#include <pmp/surface_mesh.h>

#include "volanim_utils/defs.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(warping)

// =====================================================================================================================

class RBFWarpStephan {
protected:
    // deformation centers
    std::vector<Eigen::Vector3d> _centers;

    // indices for centers
    std::vector<Eigen::Index> _indices;

    // number of rbf centers
    Eigen::Index _num_kernels = 0;

    // solver with prefactorized warp
    // was PartialPivLU
    Eigen::FullPivLU<Eigen::MatrixXd> _solver;

    // for consistency check, save number of vertices in init mesh
    Eigen::Index _n_control_points = 0;

    // for mapping meshes directly
    std::vector<pmp::IndexType> _vertex_mask = std::vector<pmp::IndexType>();
public:
    RBFWarpStephan() = default;
    virtual ~RBFWarpStephan() = default;

    // initialize new RBF warp
    auto init_warp(const pmp::SurfaceMesh& target_undeformed, size_t num_centers, const std::vector<pmp::IndexType>& masked_vertices = std::vector<pmp::IndexType>()) -> void;

    // initialize new RBF warp from positions
    auto init_warp(const Eigen::Matrix3Xd& control_points, size_t num_kernels) -> void;

    // apply warp to a new mesh
    // deform source mesh to match the target positions
    auto apply_warp(const pmp::SurfaceMesh& target_deformed, const pmp::SurfaceMesh& source_undeformed) -> pmp::SurfaceMesh;

    // apply warp to a list of positions
    auto apply_warp(const Eigen::Matrix3Xd& deformed_control_points, const Eigen::MatrixXd& warped_points) -> Eigen::Matrix3Xd;

protected:
    // farthest point sampling for rbf warp centers
    auto find_rbf_centers(const Eigen::Matrix3Xd& control_points, size_t num_kernels) -> void;

    // prefactorize solver
    auto prefactorize_rbf_warp() -> void;

    // convert mesh position to rbf warp positions
    static auto mesh_to_positions(const pmp::SurfaceMesh& mesh, const std::vector<pmp::IndexType>& mask = std::vector<pmp::IndexType>()) -> Eigen::Matrix3Xd;
};

// =====================================================================================================================

NAMESPACE_END(warping)
NAMESPACE_END(volumetric_base)

// =====================================================================================================================

#endif //VOLUMETRIC_BASE_RBF_WARP_STEPHAN_H

// =====================================================================================================================
