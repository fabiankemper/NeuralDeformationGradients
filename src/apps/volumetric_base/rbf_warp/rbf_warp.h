// =====================================================================================================================
// Created by fabian on 4/9/25.
// mail@fabiankemper.de
// =====================================================================================================================

#pragma once
#ifndef VOLUMETRIC_BASE_RBF_WARP_H
#define VOLUMETRIC_BASE_RBF_WARP_H

// =====================================================================================================================

#include <Eigen/Eigen>
#include <pmp/surface_mesh.h>

#include "volanim_utils/defs.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(warping)

// =====================================================================================================================

class RBFWarp {
public:
    // which polynomial should this rbf support
    enum class Poly
    {
        RBFOnly,
        Linear, // {1, x, y, z}
        Quadratic, // {1, x, y, z, xx, yy, zz, xy, yz, xz}
    };
protected:
    // polynomial
    Poly _poly { Poly::Linear };

    // deformation centers -> C
    Eigen::Matrix3Xd _kernels;
    // all control points -> P
    Eigen::Matrix3Xd _control_points;

    // indices for centers
    std::vector<Eigen::Index> _indices;

    // cached blocks for the left system
    // Cached blocks for the left system:
    // K = Φ(P_, C_)   (M x K)
    // Pc = P(P_)      (M x t)
    // Rc = P(C_)^T    (t x K)  [for linear/quadratic; zeros if RBFOnly]
    Eigen::MatrixXd _K;           // M x K, top left
    Eigen::MatrixXd _Pc;          // M x t, top right
    Eigen::MatrixXd _Cp;          // t x K  (== P(C_)^T), bottom left
    Eigen::MatrixXd _Left;        // (M+t) x (K+t)  == [K Pc; Rc 0]

    // pre-factorize solver (placeholder)
    Eigen::HouseholderQR<Eigen::MatrixXd> _qr_solver;
    Eigen::LDLT<Eigen::MatrixXd> _ldlt_solver;

    // for mapping meshes directly
    std::vector<pmp::IndexType> _vertex_mask = std::vector<pmp::IndexType>();
public:

    explicit RBFWarp(Poly poly = Poly::Quadratic) : _poly(poly) {}
    virtual ~RBFWarp() = default;

    // initialize new RBF warp
    auto init_warp(const pmp::SurfaceMesh& target_undeformed, size_t num_centers, const std::vector<pmp::IndexType>& masked_vertices = std::vector<pmp::IndexType>()) -> void;

    // initialize new RBF warp from positions
    auto init_warp(const Eigen::Matrix3Xd& control_points, size_t num_kernels) -> void;

    // apply warp to a new mesh
    // deform source mesh to match the target positions
    auto apply_warp(const pmp::SurfaceMesh& target_deformed, const pmp::SurfaceMesh& source_undeformed) -> pmp::SurfaceMesh;

    // apply warp to a list of positions
    auto apply_warp(const Eigen::Matrix3Xd& deformed_control_points, const Eigen::Matrix3Xd& warp_points) -> Eigen::Matrix3Xd;

protected:
    // farthest point sampling for rbf warp centers
    auto find_rbf_centers(size_t num_kernels) -> void;

    // get pairwise kernel
    auto pairwise_kernel(const Eigen::Matrix3Xd& A, const Eigen::Matrix3Xd& B) -> Eigen::MatrixXd;

    // convert mesh position to rbf warp positions
    static auto mesh_to_positions(const pmp::SurfaceMesh& mesh, const std::vector<pmp::IndexType>& mask = std::vector<pmp::IndexType>()) -> Eigen::Matrix3Xd;

    // kernel function
    inline auto phi(double r) const
    {
        // triharmonic
        return r * r * r;
    }

    // how many rows at border
    auto tail_terms() const -> long
    {
        switch (_poly) {
            case Poly::RBFOnly: return 0;
            case Poly::Linear: return 4;
            case Poly::Quadratic: return 10;
        }
        return 0;
    }

    // build polynomial
    auto build_P(const Eigen::Matrix3Xd& points) const -> Eigen::MatrixXd;

    // build left side - use internal cache variables
    auto build_left_matrix() -> void;


};

// =====================================================================================================================

NAMESPACE_END(warping)
NAMESPACE_END(volumetric_base)

// =====================================================================================================================

#endif //VOLUMETRIC_BASE_RBF_WARP_H

// =====================================================================================================================
