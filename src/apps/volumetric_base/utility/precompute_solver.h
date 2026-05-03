#pragma once
#ifndef PRECOMPUTESOLVER_H
#define PRECOMPUTESOLVER_H

#ifdef USE_MKL
#include <Eigen/PardisoSupport>
#endif

#include <functional>
#include <vector>
#include "volumetric_base/utility/matrix_util.hpp"

class PrecomputeSolver {
public:

    PrecomputeSolver()= default;
    ~PrecomputeSolver() = default;
    PrecomputeSolver& operator=(const PrecomputeSolver& other) {
        if(this==&other){ return *this;}
        left_idx = other.left_idx;
        return *this;
    }

    void reset() {
        precomputed=false;
    }

    const matrix_util::DenseMatrix& solve(
        const matrix_util::SparseMatrix& A,
        const Eigen::MatrixXd& B,
        const std::function<bool(unsigned int)>& is_constrained,
        const Eigen::MatrixXd& C
    );

    int num_cols{};
    int n_unconstrained{};
    std::vector<int> left_idx;
    std::vector<int> idx;
    
    Eigen::MatrixXf BB, XX;
    Eigen::MatrixXd X;

    // NEW: Persistent caching buffers to eliminate heap allocations during the hot loop
    Eigen::MatrixXf C_float;
    Eigen::MatrixXf rhs_penalty;

#ifdef USE_MKL
    using MklIndex = long long;
    Eigen::PardisoLLT<Eigen::SparseMatrix<float, Eigen::ColMajor, MklIndex>> solver;
    Eigen::SparseMatrix<float, Eigen::ColMajor, MklIndex> rhs_update;
#else
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>, Eigen::Upper> solver;
    Eigen::SparseMatrix<float> rhs_update;
#endif

    bool precomputed = false;
};
#endif //PRECOMPUTESOLVER_H