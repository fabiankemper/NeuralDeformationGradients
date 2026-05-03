// =====================================================================================================================
//
// Created by fabian on 13/11/2025.
//
// =====================================================================================================================

#pragma once
#ifndef VOLUMETRIC_ANIMATION_EIGEN_SPARSE_HPP
#define VOLUMETRIC_ANIMATION_EIGEN_SPARSE_HPP

// =====================================================================================================================

#include <filesystem>
#include <type_traits>

#include <Eigen/Sparse>

#include "volanim_utils/defs.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volanim::utils)

// =====================================================================================================================

// --- eigenSparse -> NPZ
template<typename T>
requires (std::is_same_v<T, float> || std::is_same_v<T, double>
        || std::is_same_v<T, int> || std::is_same_v<T, long>
    )
auto eigen_sparse_to_npz(Eigen::SparseMatrix<T> mat, const std::filesystem::path& filename) -> void;

extern template auto eigen_sparse_to_npz<double>(Eigen::SparseMatrix<double> mat, const std::filesystem::path& filename) -> void;
extern template auto eigen_sparse_to_npz<float> (Eigen::SparseMatrix<float> mat,  const std::filesystem::path& filename) -> void;
extern template auto eigen_sparse_to_npz<int>   (Eigen::SparseMatrix<int> mat,    const std::filesystem::path& filename) -> void;
extern template auto eigen_sparse_to_npz<long>  (Eigen::SparseMatrix<long> mat,   const std::filesystem::path& filename) -> void;



// --- NPZ -> eigenSparse
template<typename T>
requires (std::is_same_v<T, float> || std::is_same_v<T, double>
    || std::is_same_v<T, int> || std::is_same_v<T, long>
)
auto eigen_sparse_from_npz(const std::filesystem::path& filename) -> Eigen::SparseMatrix<T>;

extern template auto eigen_sparse_from_npz<double>(const std::filesystem::path& filename) -> Eigen::SparseMatrix<double>;
extern template auto eigen_sparse_from_npz<float> (const std::filesystem::path& filename) -> Eigen::SparseMatrix<float>;
extern template auto eigen_sparse_from_npz<int>   (const std::filesystem::path& filename) -> Eigen::SparseMatrix<int>;
extern template auto eigen_sparse_from_npz<long>  (const std::filesystem::path& filename) -> Eigen::SparseMatrix<long>;


// =====================================================================================================================

NAMESPACE_END(volanim::utils)

// =====================================================================================================================

#endif //VOLUMETRIC_ANIMATION_EIGEN_SPARSE_HPP

// =====================================================================================================================
