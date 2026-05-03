// =====================================================================================================================
// Created by fabian on 4/9/25.
// mail@fabiankemper.de
// =====================================================================================================================

#include "rbf_warp_stephan.h"

#include <limits>
#include <fstream>

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(warping)

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

auto RBFWarpStephan::init_warp(
    const pmp::SurfaceMesh& target_undeformed,
    const size_t num_centers,
    const std::vector<pmp::IndexType>& masked_vertices
) -> void
{
    // set vertex mask
    _vertex_mask = masked_vertices;

    // convert mesh vertices to position
    const Eigen::Matrix3Xd positions = mesh_to_positions(target_undeformed, _vertex_mask);

    // call actual method
    init_warp(positions, num_centers);
}

// ---------------------------------------------------------------------------------------------------------------------

auto RBFWarpStephan::init_warp(const Eigen::Matrix3Xd& control_points, const size_t num_kernels) -> void
{
    // total number of centers available in target
    _n_control_points = control_points.cols();

    // find rbf warp centers
    find_rbf_centers(control_points, num_kernels);

    // pre-factorize solver matrix
    prefactorize_rbf_warp();
}

// ---------------------------------------------------------------------------------------------------------------------

auto RBFWarpStephan::apply_warp(const pmp::SurfaceMesh& target_deformed, const pmp::SurfaceMesh& source_undeformed) -> pmp::SurfaceMesh
{
    // get deformed control points
    const auto deformed_target_points = mesh_to_positions(target_deformed, _vertex_mask);
    const auto source_points = mesh_to_positions(source_undeformed, {});

    const auto source_deformed_points = apply_warp(deformed_target_points, source_points);

    // create result
    pmp::SurfaceMesh source_deformed = source_undeformed;

    // set positions
    for (auto vertex : source_deformed.vertices()) {
        source_deformed.position(vertex) = source_deformed_points.col(vertex.idx());
    }

    return source_deformed;
}

// ---------------------------------------------------------------------------------------------------------------------

auto RBFWarpStephan::apply_warp(const Eigen::Matrix3Xd& deformed_control_points, const Eigen::MatrixXd& warped_points) -> Eigen::Matrix3Xd
{
    // check preconditions
    if (_num_kernels < 5) {
        throw std::runtime_error("Too few center for rbf warp.");
    }

    if (_n_control_points != deformed_control_points.cols()) {
        throw std::runtime_error("RBFWarp: Target mesh has different vertex count than initialized mesh.");
    }

    const auto n_centers = static_cast<Eigen::Index> (_centers.size());

    // setup Right-hand side
    Eigen::MatrixXd B(n_centers + 4, 3); // B.setZero();

    for (Eigen::Index i = 0; i < n_centers; ++i) {
        // which vertex?
        const auto idx = _indices[i];

        // calculate displacement for center / vertex
        Eigen::Vector3d displacement = deformed_control_points.col(idx) - _centers[i];

        // set right value
        // B.block(i, 0, 1, 3) = Eigen::Vector3d(displacement);

        for (Eigen::Index j = 0; j < 3; ++j) {
            B(i, j) = displacement[j];
        }
    }

    // last rows
    B.block(/*startRow=*/n_centers, /*startCol=*/0, /*blockRows=*/4, /*blockCols=*/3).setZero();

    // solve system
    Eigen::MatrixXd X(3, n_centers + 4);

    // #pragma omp parallel for default(none) shared(X, B) schedule(static)
    for (int k = 0; k < 3; ++k) {
        X.row(k) = (_solver.solve(B.col(k))).transpose();
    }

    // copy input mesh as result mesh
    Eigen::Matrix3Xd result = warped_points;

    // apply solution
    // #pragma omp parallel for default(none) shared(result, X, n_centers, source) schedule(static)
    for (Eigen::Index idx = 0; idx < warped_points.cols(); ++idx) {
        // current position
        Eigen::Vector3d p = warped_points.col(idx);
        Eigen::Vector3d f;
        f.setZero();

        // calculate weights
        for (Eigen::Index i = 0; i < n_centers; ++i) {
            double r = (p - _centers[i]).norm();
            f += X.col(i).transpose() * r * r * r;
        }

        // linear polynomial
        f += X.col(n_centers + 0).transpose();
        f += X.col(n_centers + 1).transpose() * p[0];
        f += X.col(n_centers + 2).transpose() * p[1];
        f += X.col(n_centers + 3).transpose() * p[2];

        // apply displacement from warp
        result.col(idx) += f;
    }

    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto RBFWarpStephan::find_rbf_centers(
    const Eigen::Matrix3Xd& control_points,
    const size_t num_kernels
) -> void
{
    const Eigen::Index n_positions = control_points.cols();
    _centers.resize(n_positions);
    _indices.resize(n_positions);
    _num_kernels = std::min(static_cast<Eigen::Index> (num_kernels), n_positions);

    std::vector<double> dist(n_positions, std::numeric_limits<double>::max());

    // init centers & indices
    for (Eigen::Index index = 0; index < n_positions; ++index) {
        _centers[index] = control_points.col(index);
        _indices[index] = index;
    }

    // k current index, we always add the 0 vertex
    for (auto k = 0; k < _num_kernels; ++k) {
        // use next (k-th) center
        Eigen::Vector3d p = _centers[k];
        dist[k] = 0.0f; // no distance at all

        // k is one bigger now!

        // maximum index for neighbors
        size_t imax = k + 1;
        double dmax = 0.0f; // current maximal distance from k

        // iterate over all rest vertices and search the max distance
        for (auto i = k + 1; i < n_positions; ++i) {
            // distance from current center
            double d = (p - _centers[i]).squaredNorm();

            // update dist[i] as shortest distance to selected centers
            if (d < dist[i]) {
                dist[i] = d;
            }

            // find the farthest center
            if (dist[i] > dmax) {
                dmax = dist[i];
                imax = i;
            }
        }

        // move the farthest center to position k
        std::swap(_centers[k+1], _centers[imax]);
        std::swap(_indices[k+1], _indices[imax]);
        std::swap(dist[k+1], dist[imax]);
    }

    // do not store unsorted/unused indices
    _centers.resize(num_kernels);
    _indices.resize(num_kernels);

    // write center positions as point cloud
    // std::ofstream out("out/centers_pc.txt");
    // for (const auto& point : _centers) {
    //     out << point[0] << " " << point[1] << " " << point[2] << "\n";
    // }
    // out.close();
}

// ---------------------------------------------------------------------------------------------------------------------

auto RBFWarpStephan::prefactorize_rbf_warp() -> void
{
    const auto n_centers = static_cast<Eigen::Index> (_centers.size());
    if (n_centers < 5) {
        throw std::runtime_error("Too few centers for rbf warp.");
    }

    // setup matrix
    Eigen::MatrixXd A(n_centers + 4, n_centers + 4);
    A.setZero();

    // setup distance matrix
    for (Eigen::Index i = 0; i < n_centers; ++i) {
        for (Eigen::Index j = 0; j < n_centers; ++j) {
            const double r = (_centers[i] - _centers[j]).norm();
            // kernel function, r^3
            A(i, j) = r * r * r;
        }
    }

    // add centers in last rows and columns
    for (Eigen::Index i = 0; i < n_centers; ++i) {
        const auto j = n_centers;
        A(i, j) = A(j, i) = 1.0;
        A(i, j + 1) = A(j + 1, i) = _centers[i][0];
        A(i, j + 2) = A(j + 2, i) = _centers[i][1];
        A(i, j + 3) = A(j + 3, i) = _centers[i][2];
    }

    // bottom right part should be initialized as zeros
    A.block(n_centers, n_centers, 4, 4).setZero();

    // small positive diagonal to help mathematical stability
    A.diagonal().head(n_centers).array() += 1e-12; // * L^3 with L defining the square bounding box

    // prefactorize rbf-matrix
    _solver.compute(A);
}

// ---------------------------------------------------------------------------------------------------------------------

auto RBFWarpStephan::mesh_to_positions(const pmp::SurfaceMesh& mesh, const std::vector<pmp::IndexType>& mask) -> Eigen::Matrix3Xd
{
    // copy mesh for modification
    pmp::SurfaceMesh mesh_copy = mesh;

    // mark vertices for ignore
    auto masked = mesh_copy.vertex_property<bool>("v:rbf_ignore");
    size_t total_masked_vertices = 0;
    for (const auto masked_idx : mask) {
        masked[pmp::Vertex(masked_idx)] = true;
        total_masked_vertices++;
    }

    // resulting positions
    Eigen::Matrix3Xd positions = Eigen::Matrix3Xd::Zero(3, static_cast<Eigen::Index>(mesh_copy.n_vertices() - total_masked_vertices));
    Eigen::Index last_index = 0;
    for (Eigen::Index index = 0; index < static_cast<Eigen::Index> (mesh_copy.n_vertices()); ++index) {
        if (masked[pmp::Vertex(index)]) {
            continue;
        }

        // copy positions for pmp
        positions.col(last_index) = Eigen::Vector3f (mesh_copy.position(pmp::Vertex(index))).cast<double>().eval();
        last_index++;
    }

    return positions;
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(warping)
NAMESPACE_END(volumetric_base)

// =====================================================================================================================
