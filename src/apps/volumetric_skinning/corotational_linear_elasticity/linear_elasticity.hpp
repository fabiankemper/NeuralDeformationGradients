// =====================================================================================================================

#pragma once

// =====================================================================================================================

#include <Eigen/Core>

#include "volumetric_base/diffgeo/prism_diffgeo.h"

// =====================================================================================================================

inline Eigen::Matrix<double, 12, 12> stack_rotation_matrix(const Eigen::Matrix3d& rota)
{
    Eigen::Matrix<double, 12, 12> res;
    res.setZero();
    res.block<3, 3>(0, 0) = rota;
    res.block<3, 3>(3, 3) = rota;
    res.block<3, 3>(6, 6) = rota;
    res.block<3, 3>(9, 9) = rota;

    return res;
}

// ---------------------------------------------------------------------------------------------------------------------

template<int N>
inline Eigen::Matrix<double, N * 3, N * 3> stack_rotation_matrix(const Eigen::Matrix3d& rota)
{
    Eigen::Matrix<double, N * 3, N * 3> res;
    res.setZero();
    for (int i = 0; i < N; i++) {
        res.block(i * 3, i * 3, 3, 3) = rota;
    }

    return res;
}

// ---------------------------------------------------------------------------------------------------------------------

inline double computeTetrahedronVolume(
    const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, const Eigen::Vector3d& p4
) {
    // Compute vectors for the edges of the tetrahedron
    Eigen::Vector3d v1 = p2 - p1;
    Eigen::Vector3d v2 = p3 - p1;
    Eigen::Vector3d v3 = p4 - p1;

    // Compute the scalar triple product
    double volume = std::abs(v1.dot(v2.cross(v3))) / 6.0;

    return volume;
}

// ---------------------------------------------------------------------------------------------------------------------

inline Eigen::Matrix<double,6,6> getHookeMatrix(double E, double nu) {
    const double lam = E*nu / ((1+nu)*(1-2*nu));
    const double G   = E / (2*(1+nu));

    Eigen::Matrix<double,6,6> D = Eigen::Matrix<double,6,6>::Zero();
    D(0,0)=lam+2*G; D(0,1)=lam;     D(0,2)=lam;
    D(1,0)=lam;     D(1,1)=lam+2*G; D(1,2)=lam;
    D(2,0)=lam;     D(2,1)=lam;     D(2,2)=lam+2*G;
    D(3,3)=G;       D(4,4)=G;       D(5,5)=G;
    return D;
}

// ---------------------------------------------------------------------------------------------------------------------

inline auto compute_k_matrix_mueller(
    const Eigen::Vector3d& x0, const Eigen::Vector3d& x1, const Eigen::Vector3d& x2, const Eigen::Vector3d& x3,
    double E, double v, Eigen::Matrix<double, 12, 12>& res
) -> void
{
    Eigen::Matrix3d X;
    X.col(0) = x1 - x0;
    X.col(1) = x2 - x0;
    X.col(2) = x3 - x0;

    double V = X.determinant();
    Eigen::Matrix3d Xinv = X.inverse();

    Eigen::Vector3d y0 = -Xinv.row(0) - Xinv.row(1) - Xinv.row(2);
    Eigen::Vector3d y1 = Xinv.row(0);
    Eigen::Vector3d y2 = Xinv.row(1);
    Eigen::Vector3d y3 = Xinv.row(2);
    std::vector ys = {y0, y1, y2, y3};

    double factor = V * E / ((1 + v) * (1 - 2 * v));
    double a = (1 - v) * factor;
    double b = v * factor;
    double c = (1 - 2 * v) * factor;

    Eigen::Matrix3d K1center;
    K1center << a, b, b,
            b, a, b,
            b, b, a;
    Eigen::Matrix3d K2center = c * Eigen::Matrix3d::Identity();

    auto cross_derivative_matrix = [](const Eigen::Vector3d& y) -> Eigen::Matrix3d {
        Eigen::Matrix3d m;
        m << 0, -y.z(), y.y(),
                y.z(), 0, -y.x(),
                -y.y(), y.x(), 0;
        return m;
    };

    auto get_ij_part = [&](int i, int j)-> Eigen::Matrix3d {
        Eigen::Matrix3d K1 = ys[i].asDiagonal() * K1center * ys[j].asDiagonal();
        Eigen::Matrix3d K2 = cross_derivative_matrix(ys[i]) * K2center * cross_derivative_matrix(ys[j]).transpose();
        return K1 + K2;
    };

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            res.block<3, 3>(3 * i, 3 * j) = get_ij_part(i, j);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

static const Eigen::Matrix<double, 3, 4> coeffs = [] {
    Eigen::Matrix<double, 3, 4> coeffs;
    coeffs << 1, 0, 0, -1,
            0, 1, 0, -1,
            0, 0, 1, -1;
    return coeffs;
}();

// ---------------------------------------------------------------------------------------------------------------------

inline Eigen::Matrix<double, 3, 4> computeTetrahedronGradients(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, const Eigen::Vector3d& d
) {
    Eigen::Matrix3d edges;
    edges.row(0) = a - d;
    edges.row(1) = b - d;
    edges.row(2) = c - d;

    return edges.inverse() * coeffs;
}

// ---------------------------------------------------------------------------------------------------------------------

inline Eigen::Matrix<double, 6, 12> getBMatrix(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, const Eigen::Vector3d& d
) {
    Eigen::Matrix<double, 6, 12> B;
    B.setZero();
    Eigen::Matrix<double, 3, 4> grads = computeTetrahedronGradients(a, b, c, d);
    Eigen::Vector3d grada = grads.col(0);
    Eigen::Vector3d gradb = grads.col(1);
    Eigen::Vector3d gradc = grads.col(2);
    Eigen::Vector3d gradd = grads.col(3);
    // gradient fill-in
    auto fill = [&](Eigen::Vector3d& grad, int offset) -> void {
        B(0, 0 + offset) = grad[0];
        B(1, 1 + offset) = grad[1];
        B(2, 2 + offset) = grad[2];

        B(3, 0 + offset) = grad[1];
        B(3, 1 + offset) = grad[0];

        B(4, 1 + offset) = grad[2];
        B(4, 2 + offset) = grad[1];

        B(5, 0 + offset) = grad[2];
        B(5, 2 + offset) = grad[0];
    };
    fill(grada, 0);
    fill(gradb, 3);
    fill(gradc, 6);
    fill(gradd, 9);
    return B;
}

inline Eigen::Matrix<double,6,18> getPrismBMatrix(const Eigen::Matrix<double,3,6>& rest_points) {
    Eigen::Matrix<double,3,6> grads  = prism_diffgeo::prism_gradient(rest_points);
    Eigen::Matrix<double, 6,18> B;
    B.setZero();
    Eigen::Vector3d grada = grads.col(0);
    Eigen::Vector3d gradb = grads.col(1);
    Eigen::Vector3d gradc = grads.col(2);
    Eigen::Vector3d gradd = grads.col(3);
    Eigen::Vector3d grade = grads.col(4);
    Eigen::Vector3d gradf = grads.col(5);

    auto fill = [&](Eigen::Vector3d& grad, int offset) -> void {
        B(0, 0 + offset) = grad[0];
        B(1, 1 + offset) = grad[1];
        B(2, 2 + offset) = grad[2];

        B(3, 0 + offset) = grad[1];
        B(3, 1 + offset) = grad[0];

        B(4, 1 + offset) = grad[2];
        B(4, 2 + offset) = grad[1];

        B(5, 0 + offset) = grad[2];
        B(5, 2 + offset) = grad[0];
        //fill first shape func
    };
    fill(grada, 0);
    fill(gradb, 3);
    fill(gradc, 6);
    fill(gradd, 9);
    fill(grade, 12);
    fill(gradf, 15);

    return B;
}
inline double tetVol(const Eigen::Vector3d& a,
                     const Eigen::Vector3d& b,
                     const Eigen::Vector3d& c,
                     const Eigen::Vector3d& d) {
    return ((b-a).cross(c-a)).dot(d-a) / 6.0;
}



inline Eigen::Matrix<double,18,18> compute_prism_stiffness_matrix(const Eigen::Matrix<double,3,6>& prism, const double youngs_mod, const double nu) {
    double volume = std::abs(prism_diffgeo::prism_volume(prism));
    Eigen::Matrix<double, 6, 6> E = getHookeMatrix(youngs_mod, nu);
    Eigen::Matrix<double, 6, 18> Be = getPrismBMatrix(prism);
    return volume*Be.transpose()*E*Be;
}

// ---------------------------------------------------------------------------------------------------------------------

inline Eigen::Matrix<double, 12, 12> compute_stiffness_matrix_K(
    const Eigen::Vector3d& a, const Eigen::Vector3d& b,
    const Eigen::Vector3d& c, const Eigen::Vector3d& d,
    double youngs, const double nu
) {
    using namespace std;
    double volume = computeTetrahedronVolume(a, b, c, d);
    Eigen::Matrix<double, 6, 6> E = getHookeMatrix(youngs, nu);
    Eigen::Matrix<double, 6, 12> Be = getBMatrix(a, b, c, d);
    return volume * Be.transpose() * E * Be;
}

// ---------------------------------------------------------------------------------------------------------------------
