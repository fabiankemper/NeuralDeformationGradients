// =====================================================================================================================
#pragma once
// =====================================================================================================================

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Sparse>

#ifdef USE_MKL
#include <Eigen/PardisoSupport>
#endif
#include <Eigen/Eigenvalues>
#include <pmp/exceptions.h>
#include <unsupported/Eigen/MatrixFunctions>

#include "volanim_utils/defs.h"

// =====================================================================================================================

NAMESPACE_BEGIN(matrix_util)

// =====================================================================================================================

typedef Eigen::Triplet<double> Triplet;

using SparseMatrix = Eigen::SparseMatrix<double>;
using DenseMatrix = Eigen::MatrixXd;

// =====================================================================================================================

inline Eigen::Matrix3d createMatrixFromVectors(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2,
                                               const Eigen::Vector3d& v3)
{
    Eigen::Matrix3d matrix;
    matrix.row(0) = v1.transpose();
    matrix.row(1) = v2.transpose();
    matrix.row(2) = v3.transpose();
    return matrix;
}

template<int w, int h>
void blockAssign(int row_start, int col_start, Eigen::Matrix<double, w, h>& addMatrix, std::vector<Triplet>& triplets)
{
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
                triplets.emplace_back(Triplet(row_start + i, col_start + j, addMatrix(i, j)));
            
        }
    }
}

template<int w, int h>
void triangularBlockAssign(int row_start, int col_start, const Eigen::Matrix<double, w, h>& addMatrix,
                           std::vector<Triplet>& triplets)
{
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            if (row_start + i <= col_start + j) {
                triplets.emplace_back(Triplet(row_start + i, col_start + j, addMatrix(i, j)));
            }
        }
    }
}

template<int w, int h>
void triangularBlockAssignWithOffsets(int row_start, int col_start, const Eigen::Matrix<double, w, h>& addMatrix,
                           std::vector<Triplet>& triplets, int offset)
{
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            if (row_start + i <= col_start + j) {
                triplets[offset++] = (Triplet(row_start + i, col_start + j, addMatrix(i, j)));
            }
        }
    }
}


template<int w, int h>
void virtualTriangularBlockAssign(int row_start, int col_start,
    std::vector<double*>& sparse_mat_entries, Eigen::SparseMatrix<double>&  mat, int & offset)
{
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            if (row_start + i <= col_start + j) {
                sparse_mat_entries[offset]=&mat.coeffRef(row_start+i, col_start + j);
                offset++;
            }
        }
    }
}

template<int w, int h>
void triangularBlockAssign(int row_start, int col_start, const Eigen::Matrix<double, w, h>& addMatrix,
                           std::vector<Triplet>& triplets, int& offset)
{
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            if (row_start + i <= col_start + j) {
                triplets[offset] = (Triplet(row_start + i, col_start + j, addMatrix(i, j)));
                offset++;
            }
        }
    }
}

template<int w, int h>
void triangularBlockAssignIntoPointers(int row_start, int col_start, const Eigen::Matrix<double, w, h>& addMatrix,
                                       int& offset, std::vector<double*>& pointers)
{
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            if (row_start + i <= col_start + j) {
                *pointers[offset] += addMatrix(i, j);
                offset++;
            }
        }
    }
}



template<int w, int h>
void triangularBlockAssignSetupForPointers(int row_start, int col_start, std::vector<double*>& pointers, Eigen::SparseMatrix<double>&  mat)
{
    for (int i = 0; i < w; i++) {
        for (int j = 0; j < h; j++) {
            if (row_start + i <= col_start + j) {
                pointers.emplace_back(&mat.coeffRef(row_start + i, col_start + j));
            }
        }
    }
}


inline void setSparseMatrixToZero(const Eigen::SparseMatrix<double>& mat) {
    // Loop over the outer dimension (columns for a column-major matrix)
    for (int k = 0; k < mat.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(mat, k); it; ++it) {
            it.valueRef() = 0.0;
        }
    }
}

inline Eigen::Matrix<double, 4, 3> retrievePositionsAsMatrix(const int a, const int b, const int c, const int d,
                                                             const Eigen::Matrix<double, -1, 3>& positions)
{
    Eigen::Matrix<double, 4, 3> result;
    result.row(0) = positions.row(a);
    result.row(1) = positions.row(b);
    result.row(2) = positions.row(c);
    result.row(3) = positions.row(d);
    return result;
}

inline void retrievePositions(int a, int b, int c, int d,
                              const Eigen::Matrix<double, -1, 3>& positions,
                              Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3, Eigen::Vector3d& p4)
{
    p1 = positions.row(a);
    p2 = positions.row(b);
    p3 = positions.row(c);
    p4 = positions.row(d);
}

template<typename T>
std::tuple<Eigen::Quaternion<T>, Eigen::Vector3<T> > get_rotation_and_translation(Eigen::Matrix<T, 4, 4> transform)
{
    Eigen::Matrix<T, 3, 3> rota = transform.block(0, 0, 3, 3);
    Eigen::Vector3<T> translation = transform.block(0, 3, 3, 1);
    return {Eigen::Quaternion<T>(rota), translation};
}



// =====================================================================================================================
inline Eigen::VectorXd flatten(const Eigen::MatrixXd& input) {
    Eigen::VectorXd res(input.rows()*input.cols());
    for (int i = 0; i < input.rows(); i++) {
        for (int j = 0; j < input.cols(); j++) {
            res(i * input.cols() + j) = input(i, j);
        }
    }
    return res;
}
template <int n>
Eigen::Matrix<double,9,n*3> vectorize_gradient(const Eigen::Matrix<double,3,n>& localGrad ) {
    Eigen::Matrix<double, 9, n * 3> G = Eigen::Matrix<double, 9, n * 3>::Zero();

    for (int i = 0; i < 3; ++i) {           // row in localGrad
        for (int j = 0; j < n; ++j) {       // column in localGrad
            int col_offset = j * 3;
            for (int k = 0; k < 3; ++k) {   // for 3D vector components
                G(i * 3 + k, col_offset + k) = localGrad(i, j);
            }
        }
    }

    return G;
}


inline Eigen::VectorXd flatten_other_way(const Eigen::MatrixXd& input) {
    Eigen::VectorXd res(input.rows()*input.cols());
    for (int i = 0; i < input.rows(); i++) {
        for (int j = 0; j < input.cols(); j++) {
            res(i + j*input.rows()) = input(i, j);
        }
    }
    return res;
}

//======================================================================================================================
inline Eigen::MatrixXd expand(const Eigen::VectorXd& input, int rows, int cols) {
    assert(input.rows() == rows*cols);
    Eigen::MatrixXd res(rows, cols);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {

            res(i,j) = input(i * cols + j);
        }
    }
    return res;
}

template <int n>
Eigen::Matrix<double, n,n> psd_projection_to_abs(const Eigen::Matrix<double, n, n> &X) {
    // Proposed by Chen et al. 2024 Stabler Neo-Hookean Simulation: Absolute Eigenvalue Filtering for Projected Newton
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, n, n> > solver(X);
    Eigen::VectorXd eigenvalues = solver.eigenvalues();
    Eigen::MatrixXd eigenvectors = solver.eigenvectors();
    for (int i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues(i) < 0) {
            eigenvalues(i) = abs(eigenvalues(i));
        }
    }
    return eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
}

template <int n>
Eigen::Matrix<double, n,n> psd_projection_to_zero(const Eigen::Matrix<double, n, n> &X) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, n, n> > solver(X);
    Eigen::VectorXd eigenvalues = solver.eigenvalues();
    Eigen::MatrixXd eigenvectors = solver.eigenvectors();
    for (int i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues(i) < 0) {
            eigenvalues(i) =0;
        }
    }
    return eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
}

NAMESPACE_END(matrix_util)

// =====================================================================================================================
