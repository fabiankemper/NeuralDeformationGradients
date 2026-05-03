// =====================================================================================================================
// Created by fabian on 4/29/25.
// mail@fabiankemper.de
// =====================================================================================================================

#pragma once
#ifndef NPY_UTILS_H
#define NPY_UTILS_H

// =====================================================================================================================

#include <filesystem>

#include "npy.hpp"
#include "Eigen/Dense"

#include "defs.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volanim::utils)

// =====================================================================================================================


// zero-copy string_view buffer for converting strings to numpy without copying
class StringViewBuf final : public std::streambuf {
public:
    // Construct from a string_view
    StringViewBuf(std::string_view string_view) {
        // 'const_cast' is safe here because we are only reading
        char* start = const_cast<char*>(string_view.data());
        char* end = start + string_view.size();

        // Set the "get" area of the buffer to the string_view's data
        this->setg(start, start, end);
    }
};

// =====================================================================================================================

// convert Eigen matrices to NPY
auto EigenDMat2NPY(const Eigen::MatrixXd& mat) -> npy::npy_data<double>;
auto EigenFMat2NPY(const Eigen::MatrixXf& mat) -> npy::npy_data<float>;

auto eigen_to_npy_file_f(const Eigen::MatrixXf& mat, const std::filesystem::path& filename) -> void;
auto eigen_to_npy_file_d(const Eigen::MatrixXd& mat, const std::filesystem::path& filename) -> void;

auto npy_file_to_eigen_matf(const std::filesystem::path& filename) -> Eigen::MatrixXf;
auto npy_file_to_eigen_matd(const std::filesystem::path& filename) -> Eigen::MatrixXd;

auto npy_file_to_eigen_vecf(const std::filesystem::path& filename) -> Eigen::VectorXf;
auto npy_file_to_eigen_vecd(const std::filesystem::path& filename) -> Eigen::VectorXd;

// =====================================================================================================================


// generic template for converting eigen matrices, vectors etc. to numpy objects
template<typename Derived>
auto Eigen2NPY(const Eigen::MatrixBase<Derived>& matrix) -> npy::npy_data<typename Derived::Scalar>
{
    using Scalar = typename Derived::Scalar;
    static_assert(
        std::is_same_v<Scalar, float> ||
        std::is_same_v<Scalar, double> ||
        std::is_same_v<Scalar, int> ||
        std::is_same_v<Scalar, int64_t>,
        "Unsupported scalar type for conversion to npy_data."
    );

    npy::npy_data<double> data;
    data.shape = {
        static_cast<uint64_t>(matrix.rows()),
        static_cast<uint64_t>(matrix.cols())
    };

    // data.data = std::vector<double>(matrix.cols() * matrix.rows());
    data.data.resize(matrix.size() * sizeof(Scalar));

    // copy data
    for(int i = 0; i < matrix.rows(); ++i) {
        for(int j = 0; j < matrix.cols(); ++j) {
            data.data[i * matrix.cols() + j] = matrix(i, j);
        }
    }

    return data;
}


// generic template for converting eigen matrices, vectors from numpy data
template<typename T>
auto to_eigen_matrix(const npy::npy_data<T>& npy_data) -> Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> {
    static_assert(
        std::is_same_v<T, float> ||
        std::is_same_v<T, double> ||
        std::is_same_v<T, int> ||
        std::is_same_v<T, int64_t>,
        "Unsupported scalar type for conversion from npy_data."
    );

    assert(npy_data.shape.size() == 2 && "NDIM must be equal to 2.");

    // map the data as a RowMajor matrix
    Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> map(
        npy_data.data.data(), npy_data.shape[0], npy_data.shape[1]
    );

    // copy into a default (column-major) matrix
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> result = map;

    return result;
}

template<typename T>
auto to_eigen_vector(const npy::npy_data<T>& npy_data) -> Eigen::Vector<T, Eigen::Dynamic>
{
    static_assert(
        std::is_same_v<T, float> ||
        std::is_same_v<T, double> ||
        std::is_same_v<T, int> ||
        std::is_same_v<T, int64_t>,
        "Unsupported scalar type for conversion from npy_data."
    );

    assert(npy_data.shape.size() == 1 && "NDIM must be equal to 1.");

    // map the data as a vector
    Eigen::Map<const Eigen::Vector<T, Eigen::Dynamic>> map(
        npy_data.data.data(), npy_data.shape[0]
    );

    // copy into a default (column-major) matrix
    Eigen::Vector<T, Eigen::Dynamic> result = map;

    return result;
}


// store numpy array in stream
template<class T>
auto npy2stream(const npy::npy_data<T>& array) -> std::string {
    std::ostringstream oss(std::ios::out | std::ios::binary);
    npy::write_npy(oss, array);
    std::string s = oss.str();
    return s;
}

// read numpy array from stream
template<class T>
auto stream2npy(const std::string& sstream) -> npy::npy_data<T>
{
    std::istringstream iss(sstream);
    auto npy_data = npy::read_npy<T>(iss);
    return npy_data;
}

template<class T>
auto stream2npy(const std::string_view& string_view) -> npy::npy_data<T>
{
    // Create the custom buffer that just views the memory
    StringViewBuf sbuf(string_view);

    // Create an input stream that reads from our custom buffer
    std::istream iss(&sbuf);

    auto npy_data = npy::read_npy<T>(iss);
    return npy_data;
}


// =====================================================================================================================

NAMESPACE_END(volanim::utils)

// =====================================================================================================================

#endif //NPY_UTILS_H

// =====================================================================================================================
