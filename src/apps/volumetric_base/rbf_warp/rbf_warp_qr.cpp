// =====================================================================================================================
// Created by fabian on 4/9/25.
// mail@fabiankemper.de
// =====================================================================================================================

#include "rbf_warp.h"

#include <limits>
#include <fstream>

#include "pmp/stop_watch.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(warping)

// =====================================================================================================================

#define USE_ALL_CONTROL_POINTS 0

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

auto RBFWarp::init_warp(
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

auto RBFWarp::init_warp(const Eigen::Matrix3Xd& control_points, const size_t num_kernels) -> void
{
    // all centers available for deformation (undeformed state)
    _control_points = control_points;

    // find rbf warp centers
    find_rbf_centers(num_kernels);

    // pre-compute left system blocks that depend only on original geometry
    build_left_matrix();
}

// ---------------------------------------------------------------------------------------------------------------------

auto RBFWarp::apply_warp(const pmp::SurfaceMesh& target_deformed, const pmp::SurfaceMesh& source_undeformed) -> pmp::SurfaceMesh
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

// classic version
auto RBFWarp::apply_warp(const Eigen::Matrix3Xd& deformed_control_points, const Eigen::Matrix3Xd& warp_points) -> Eigen::Matrix3Xd
{
    // // sample right side
#if USE_ALL_CONTROL_POINTS
    Eigen::Matrix3Xd _deformed_control_points = deformed_control_points;
#else
    Eigen::Matrix3Xd _deformed_control_points = Eigen::Matrix3Xd::Zero(3, _kernels.cols());

    for (long index = 0; index < _kernels.cols(); ++index) {
        _deformed_control_points.col(index) = deformed_control_points.col(_indices[index]);
    }
#endif

    const auto M = _control_points.cols();
    const auto K = _kernels.cols();
    const auto t = tail_terms();

    std::cout << "QR APPLY WARP\n";

    if (K < t + 1) {
        throw std::runtime_error("Too few centers for rbf warp.");
    }
    if (M != _deformed_control_points.cols()) {
        throw std::runtime_error("RBFWarp: Target mesh has different vertex count than initialized mesh.");
    }

    auto watch = pmp::StopWatch();
    watch.start();

    // 1) Build RHS as displacements ΔP = Pd - P (M x 3)
    Eigen::MatrixXd Right = Eigen::MatrixXd::Zero(M + t, 3);
    Right.topRows(M).noalias() = (_deformed_control_points - _control_points).transpose(); // M x 3

    // 2) Build b = A^T * Right  -> size (K+t) x 3
    Eigen::MatrixXd b = Right;
    std::cout << "Built right hand side.\n";

    // solve for deformation
    Eigen::MatrixXd WA = _qr_solver.solve(b); // (K+t) x 3
    std::cout << "Solved for WA.\n";

    // apply warp to query points
    const auto N = warp_points.cols();

    // split into two operations
    const Eigen::MatrixXd W = WA.topRows(K); // K x 3
    const Eigen::MatrixXd A = (t > 0) ? WA.bottomRows(t) : Eigen::MatrixXd(0, 3);
    std::cout << "A dimensions: " << A.rows() << "x" << A.cols() << "\n";

    const Eigen::MatrixXd poly_highres = build_P(warp_points);

    Eigen::Matrix3Xd result = Eigen::Matrix3Xd::Zero(3, N);

    #pragma omp parallel for schedule(static)
    for (long v_idx = 0; v_idx < N; ++v_idx) {
        result.col(v_idx) = warp_points.col(v_idx);

        for (long k_idx = 0; k_idx < K; ++k_idx) {
            result.col(v_idx) += W.row(k_idx) * phi((_kernels.col(k_idx) - warp_points.col(v_idx)).norm());
        }

        for (long t_idx = 0; t_idx < t; ++t_idx) {
            result.col(v_idx) += A.row(t_idx) * poly_highres(v_idx, t_idx);
        }
    }

    watch.stop();
    std::cout << "RBF solve took: " << watch.elapsed() << " ms\n";


    return result;



    //
    // // Φ(X, C) is N x K
    // Eigen::MatrixXd Phi_XC(N, K);
    // {
    //     // pairwise kernel expects (3xN, 3xK) and return (MxK) -> feed X as 'A'
    //     Phi_XC.noalias() = pairwise_kernel(warp_points, _kernels);
    // }
    // std::cout << "Phi_XC built.\n";
    //
    // // P_x is N x t
    // const Eigen::MatrixXd Px = (t > 0) ? build_P(warp_points) : Eigen::MatrixXd(N, 0);
    //
    // // Y = Φ(X,C) * W + P_x * A    (N x 3)
    // Eigen::MatrixXd Y = Phi_XC * W;
    // std::cout << "Y = " << Y.block(0, 0, 10, 3) << std::endl;
    // if (t > 0) {
    //     Y.noalias() += Px * A;
    // }
    // std::cout << "Y = " << Y.block(0, 0, 10, 3) << std::endl;
    // std::cout << "Px * A done.\n";
    //
    // std::cout << "W[-10]= " << W.bottomRows(10) << std::endl;
    //
    // // Y = dX -> displacement fields
    // return warp_points + Y.transpose();
}

// ---------------------------------------------------------------------------------------------------------------------

auto RBFWarp::find_rbf_centers(
    const size_t num_kernels
) -> void
{
    const Eigen::Index M = _control_points.cols();
    const Eigen::Index K = std::min<Eigen::Index>(num_kernels, M);

    _indices.clear();
    _kernels.resize(3, 0);
    if (K <= 0) {
        return;
    }

    // Seed: farthest from centroid (more stable than column 0)
    const Eigen::Vector3d centroid = _control_points.rowwise().mean();
    Eigen::Index s = 0;
    double dmax = -1.0;

    for (Eigen::Index i = 0; i < M; ++i) {
        const double d = (_control_points.col(i) - centroid).squaredNorm();
        if (d > dmax) {
            dmax = d;
            s = i;
        }
    }
    _indices.reserve(K);
    _indices.push_back(s);

    // mindist[i] = min distance^2 from point i to the selected set
    std::vector<double> mindist(M, std::numeric_limits<double>::infinity());
    for (Eigen::Index i = 0; i < M; ++i) {
        mindist[i] = (_control_points.col(i) - _control_points.col(s)).squaredNorm();
    }
    mindist[s] = 0.0;

    for (Eigen::Index k = 1; k < K; ++k) {
        // pick the furthest by current min-distance
        Eigen::Index imax = 0; double far = -1.0;
        for (Eigen::Index i = 0; i < M; ++i) {
            if (mindist[i] > far) {
                far = mindist[i];
                imax = i;
            }
        }
        _indices.push_back(imax);

        // update mindist with new center
        const Eigen::Vector3d ck = _control_points.col(imax);
        for (Eigen::Index i = 0; i < M; ++i) {
            const double d = (_control_points.col(i) - ck).squaredNorm();
            if (d < mindist[i]) {
                mindist[i] = d;
            }
        }
        mindist[imax] = 0.0;
    }

    // build centers matrix from selected indices
    _kernels.resize(3, K);
    for (Eigen::Index k = 0; k < K; ++k) {
        _kernels.col(k) = _control_points.col(_indices[k]);
    }

    // (optional) write centers
    std::ofstream out("out/centers_pc.txt");
    for (Eigen::Index k = 0; k < K; ++k) {
        const auto p = _kernels.col(k);
        out << p(0) << " " << p(1) << " " << p(2) << "\n";
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto RBFWarp::pairwise_kernel(const Eigen::Matrix3Xd& A, const Eigen::Matrix3Xd& B) -> Eigen::MatrixXd
{
    const auto M = A.cols();
    const auto K = B.cols();
    Eigen::MatrixXd result(M, K);

    #pragma omp parallel for schedule(static)
    for (auto i = 0; i < M; ++i) {
        const Eigen::Vector3d a_i = A.col(i);
        for (auto j = 0; j < K; ++j) {
            const Eigen::Vector3d b_j = B.col(j);
            const double r = (a_i - b_j).norm();
            result(i, j) = phi(r);
        }
    }
    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto RBFWarp::mesh_to_positions(const pmp::SurfaceMesh& mesh, const std::vector<pmp::IndexType>& mask) -> Eigen::Matrix3Xd
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

    #pragma omp parallel for schedule(static)
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

// build polynomal border of left side
auto RBFWarp::build_P(const Eigen::Matrix3Xd& points) const -> Eigen::MatrixXd
{
    // each row contains one polynom for a given point
    const auto M = points.cols();
    const auto t = tail_terms();

    Eigen::MatrixXd P(M, t);
    if (t == 0) {
        return P; // emtpy (0 cols)
    }

    // columns follow sample ordering
    // Linear: [1, x, y, z]
    // Quadratic: [1, x, y, z, xx, yy, zz, xy, yz, xz]
    for (long i = 0; i < M; ++i) {
        const auto x = points(0, i);
        const auto y = points(1, i);
        const auto z = points(2, i);
        long c = 0;
        if (_poly == Poly::Linear || _poly == Poly::Quadratic) {
            P(i, c++) = 1.0;
            P(i, c++) = x;
            P(i, c++) = y;
            P(i, c++) = z;
        }
        if (_poly == Poly::Quadratic) {
            P(i, c++) = x * x;
            P(i, c++) = y * y;
            P(i, c++) = z * z;
            P(i, c++) = x * y;
            P(i, c++) = y * z;
            P(i, c++) = x * z;
        }
    }
    return P;
}

// ---------------------------------------------------------------------------------------------------------------------

auto RBFWarp::build_left_matrix() -> void
{
#if not USE_ALL_CONTROL_POINTS
    _control_points = _kernels;
#endif
    const auto M = _control_points.cols(); // control points, constraints
    const auto K = _kernels.cols();        // kernels, centers
    const auto t = tail_terms();

    std::cout << "Build left side with M=" << M << " K=" << K << "\n";

    // Blocks depending only on geometry
    _K  = pairwise_kernel(_control_points, _kernels);                 // M x K
    std::cout << "_K built.\n";
    _Pc = (t > 0) ? build_P(_control_points) : Eigen::MatrixXd(M, 0); // M x t
    std::cout << "_Pc built.\n";
    _Cp = (t > 0) ? build_P(_kernels).transpose() : Eigen::MatrixXd(0, K); // t x K
    std::cout << "_Cp built.\n";

    // it is possible to sub-sample all control_points using furthest point sampling again,
    // but only on fewer lines.

    // create new left side
    _Left = Eigen::MatrixXd::Zero(M + t, K + t);
    _Left.block(0, 0, M, K) = _K;
    if (t > 0) {
        _Left.block(0, K, M, t) = _Pc; // top right
        _Left.block(M, 0, t, K) = _Cp; // bottom left
    }

    std::cout << "Left side built.\n";

    // Tikhonov regularization
    // append diagonal rows below the _Left matrix (K x K) and add a small positive value..
    // not done here

    // pre-factorize
    std::cout << "Pre-factorize ... ";
    _qr_solver = Eigen::HouseholderQR<Eigen::MatrixXd> (_Left);
    std::cout << " done.\n";
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(warping)
NAMESPACE_END(volumetric_base)

// =====================================================================================================================
