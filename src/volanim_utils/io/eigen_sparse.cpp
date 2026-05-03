//
// Created by fabian on 13/11/2025.
//
// =====================================================================================================================

#include "eigen_sparse.hpp"

#include <npy.hpp>

#include "volanim_utils/io/io_npz.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volanim::utils)

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

template<typename T>
requires (std::is_same_v<T, float> || std::is_same_v<T, double>
    || std::is_same_v<T, int> || std::is_same_v<T, long>
)
auto eigen_sparse_to_npz(Eigen::SparseMatrix<T> mat, const std::filesystem::path& filename) -> void
{
    using Index = std::int64_t;
    using CSR   = Eigen::SparseMatrix<T, Eigen::RowMajor, Index>;

    // convert matrix to row-major CSR
    CSR csr = mat;        // layout conversion
    csr.makeCompressed(); // guarantee valid indptr/indices/valuePtr

    const auto nnz   = static_cast<std::size_t>(csr.nonZeros());
    const auto nrows = static_cast<std::size_t>(csr.rows());
    const auto ncols = static_cast<std::size_t>(csr.cols());

    // create new zip file
    NPZFile npz_file(filename, ZipOpenMode::New);

    // ---------------- data ----------------
    npy::npy_data<T> data_arr;
    data_arr.fortran_order = false;
    data_arr.shape = {
        static_cast<unsigned long>(nnz)
    };
    data_arr.data.assign(csr.valuePtr(), csr.valuePtr() + nnz);

    npz_file.add_entry("data", data_arr);

    // ---------------- indices (column indices) ----------------
    npy::npy_data<Index> indices_arr;
    indices_arr.fortran_order = false;
    indices_arr.shape = {
        static_cast<unsigned long>(nnz)
    };
    indices_arr.data.assign(csr.innerIndexPtr(), csr.innerIndexPtr() + nnz);

    npz_file.add_entry("indices", indices_arr);

    // ---------------- indptr (row pointers) ----------------
    npy::npy_data<Index> indptr_arr;
    indptr_arr.fortran_order = false;
    indptr_arr.shape = {
        static_cast<unsigned long>(nrows + 1)
    };
    indptr_arr.data.assign(csr.outerIndexPtr(),
                           csr.outerIndexPtr() + (nrows + 1));

    npz_file.add_entry("indptr", indptr_arr);

    // ---------------- shape ----------------
    npy::npy_data<Index> shape_arr;
    shape_arr.fortran_order = false;
    shape_arr.shape = { 2UL };
    shape_arr.data = {
        static_cast<Index>(nrows),
        static_cast<Index>(ncols)
    };

    npz_file.add_entry("shape", shape_arr);

    // ---------------- format ----------------
    npy::npy_data<std::string> format_arr;
    format_arr.fortran_order = false;
    format_arr.shape = {};
    format_arr.data = {"csr"};

    npz_file.add_entry("format", format_arr);

    // write data
    npz_file.write();
}

// explict instantiation
template auto eigen_sparse_to_npz<double>(Eigen::SparseMatrix<double> mat, const std::filesystem::path& filename) -> void;
template auto eigen_sparse_to_npz<float> (Eigen::SparseMatrix<float> mat, const std::filesystem::path& filename) -> void;
template auto eigen_sparse_to_npz<int>   (Eigen::SparseMatrix<int> mat, const std::filesystem::path& filename) -> void;
template auto eigen_sparse_to_npz<long>  (Eigen::SparseMatrix<long> mat, const std::filesystem::path& filename) -> void;


// ---------------------------------------------------------------------------------------------------------------------

template<typename T>
requires (std::is_same_v<T, float> || std::is_same_v<T, double>
    || std::is_same_v<T, int> || std::is_same_v<T, long>
)
auto eigen_sparse_from_npz(const std::filesystem::path& filename) -> Eigen::SparseMatrix<T>
{
    using Index = std::int64_t;

    // open npz
    NPZFile npz_file(filename, ZipOpenMode::ReadOnly);

    // --- read format ---
    // todo: check why there are 2 entries
    auto format_arr = npz_file.get_entry<std::string>("format");
    if (format_arr.data.size() != 2) {
        throw std::runtime_error("Format error must have exactly one entry!");
    }
    if (format_arr.data[1] != "csr") {
        throw std::runtime_error("Currently only CSR format sparse matrices are allowed.");
    }


    // --- read data ---
    auto data_arr = npz_file.get_entry<T>("data");
    const std::size_t nnz = data_arr.data.size();
    if (data_arr.shape.size() != 1 || static_cast<std::size_t>(data_arr.shape[0]) != nnz) {
        throw std::runtime_error("eigen_sparse_from_npz: invalid 'data' shape.");
    }

    // --- read shape ---
    std::vector<Index> shape_vec;
    try {
        const auto shape_arr = npz_file.get_entry<Index>("shape");
        if (shape_arr.data.size() != 2) {
            throw std::runtime_error("eigen_sparse_from_npz: 'shape' must have length 2.");
        }
        shape_vec = shape_arr.data;
    } catch (...) {
        // try int32 as a fallback (SciPy usually uses int64 nowadays, but just in case)
        auto shape_arr32 = npz_file.get_entry<std::int32_t>("shape");
        if (shape_arr32.data.size() != 2) {
            throw std::runtime_error("eigen_sparse_from_npz: 'shape' must have length 2.");
        }
        shape_vec.assign(shape_arr32.data.begin(), shape_arr32.data.end());
    }

    const Index nrows = shape_vec[0];
    const Index ncols = shape_vec[1];

    if (nrows < 0 || ncols < 0) {
        throw std::runtime_error("eigen_sparse_from_npz: negative shape entries.");
    }

    // --- read indices (columns) ---
    std::vector<Index> indices;
    try {
        auto indices_arr = npz_file.get_entry<Index>("indices");
        if (indices_arr.shape.size() != 1 ||
            static_cast<std::size_t>(indices_arr.shape[0]) != indices_arr.data.size()) {
            throw std::runtime_error("eigen_sparse_from_npz: invalid 'indices' shape.");
        }
        indices = std::move(indices_arr.data);
    } catch (...) {
        // fallback to int32
        auto indices_arr32 = npz_file.get_entry<std::int32_t>("indices");
        if (indices_arr32.shape.size() != 1 ||
            static_cast<std::size_t>(indices_arr32.shape[0]) != indices_arr32.data.size()) {
            throw std::runtime_error("eigen_sparse_from_npz: invalid 'indices' shape (int32).");
        }
        indices.assign(indices_arr32.data.begin(), indices_arr32.data.end());
    }

    if (indices.size() != nnz) {
        throw std::runtime_error("eigen_sparse_from_npz: 'indices' size != nnz.");
    }

    // --- read indptr (row pointers) ---
    std::vector<Index> indptr;
    try {
        auto indptr_arr = npz_file.get_entry<Index>("indptr");
        indptr = std::move(indptr_arr.data);
    } catch (...) {
        // fallback to int32
        auto indptr_arr32 = npz_file.get_entry<std::int32_t>("indptr");
        indptr.assign(indptr_arr32.data.begin(), indptr_arr32.data.end());
    }

    if (indptr.size() != static_cast<std::size_t>(nrows + 1)) {
        throw std::runtime_error("eigen_sparse_from_npz: 'indptr' size != nrows+1.");
    }
    if (indptr.back() != static_cast<Index>(nnz)) {
        throw std::runtime_error("eigen_sparse_from_npz: 'indptr' last entry != nnz.");
    }

    // --- build Eigen from CSR arrays via Eigen::Map ---
    using CSRIndex = Index;
    using CSR      = Eigen::SparseMatrix<T, Eigen::RowMajor, CSRIndex>;

    const CSRIndex rows = static_cast<CSRIndex>(nrows);
    const CSRIndex cols = static_cast<CSRIndex>(ncols);
    const CSRIndex nnz_i = static_cast<CSRIndex>(nnz);

    Eigen::Map<const CSR> csr(
        rows,
        cols,
        nnz_i,
        indptr.data(),      // outer index (row pointers)
        indices.data(),     // inner index (column indices)
        data_arr.data.data() // non-zero values
    );

    // Convert to default (ColMajor, Eigen::Index) SparseMatrix<T> for general use.
    Eigen::SparseMatrix<T> mat = csr;
    return mat;
}

template auto eigen_sparse_from_npz<double>(const std::filesystem::path& filename) -> Eigen::SparseMatrix<double>;
template auto eigen_sparse_from_npz<float> (const std::filesystem::path& filename) -> Eigen::SparseMatrix<float>;
template auto eigen_sparse_from_npz<int>   (const std::filesystem::path& filename) -> Eigen::SparseMatrix<int>;
template auto eigen_sparse_from_npz<long>  (const std::filesystem::path& filename) -> Eigen::SparseMatrix<long>;


// ---------------------------------------------------------------------------------------------------------------------


// =====================================================================================================================

NAMESPACE_END(volanim::utils)

// =====================================================================================================================
