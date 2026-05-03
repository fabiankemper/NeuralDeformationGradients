// =====================================================================================================================
// Created by fabian on 4/29/25.
// mail@fabiankemper.de
// =====================================================================================================================

#include "npy_utils.h"

#include "Eigen/src/Core/util/Meta.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volanim)
NAMESPACE_BEGIN(utils)

// =====================================================================================================================



// ---------------------------------------------------------------------------------------------------------------------

auto EigenDMat2NPY(const Eigen::MatrixXd& mat) -> npy::npy_data<double>
{
    npy::npy_data<double> data;
    data.shape = {
        static_cast<unsigned long>(mat.rows()),
        static_cast<unsigned long>(mat.cols())
    };

    // data.data = std::vector<double>(mat.cols() * mat.rows());
    data.data.resize(mat.cols() * mat.rows());

    // copy data
    for(int i = 0; i < mat.rows(); ++i) {
        for(int j = 0; j < mat.cols(); ++j) {
            data.data[i * mat.cols() + j] = mat(i, j);
        }
    }

    return data;
}

// ---------------------------------------------------------------------------------------------------------------------

auto EigenDMat2NPY(const Eigen::MatrixXf& mat) -> npy::npy_data<float>
{
    npy::npy_data<float> data;
    data.shape = {
        static_cast<unsigned long>(mat.rows()),
        static_cast<unsigned long>(mat.cols())
    };

    // data.data = std::vector<float>(mat.cols() * mat.rows());
    data.data.resize(mat.cols() * mat.rows());

    // copy data
    for(int i = 0; i < mat.rows(); ++i) {
        for(int j = 0; j < mat.cols(); ++j) {
            data.data[i * mat.cols() + j] = mat(i, j);
        }
    }

    return data;
}

// ---------------------------------------------------------------------------------------------------------------------

auto eigen_to_npy_file_f(const Eigen::MatrixXf& mat, const std::filesystem::path& filename) -> void
{
    auto npy_data = EigenDMat2NPY(mat);
    std::string binary = npy2stream(npy_data);

    std::ofstream ofs(filename, std::ios::binary);
    ofs.write(binary.data(), static_cast<std::streamsize>(binary.size()));
}

// ---------------------------------------------------------------------------------------------------------------------

auto eigen_to_npy_file_d(const Eigen::MatrixXd& mat, const std::filesystem::path& filename) -> void
{
    auto npy_data = EigenDMat2NPY(mat);
    std::string binary = npy2stream(npy_data);

    std::ofstream ofs(filename, std::ios::binary);
    ofs.write(binary.data(), static_cast<std::streamsize>(binary.size()));
}

// ---------------------------------------------------------------------------------------------------------------------

auto npy_file_to_eigen_matf(const std::filesystem::path& filename) -> Eigen::MatrixXf
{
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs) {
        throw std::runtime_error("Failed to open file: " + filename.string());
    }

    const auto npy_data = npy::read_npy<float>(ifs);
    return to_eigen_matrix(npy_data);
}

// ---------------------------------------------------------------------------------------------------------------------

auto npy_file_to_eigen_matd(const std::filesystem::path& filename) -> Eigen::MatrixXd
{
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs) {
        throw std::runtime_error("Failed to open file: " + filename.string());
    }

    const auto npy_data = npy::read_npy<double>(ifs);
    return to_eigen_matrix(npy_data);
}

// ---------------------------------------------------------------------------------------------------------------------

auto npy_file_to_eigen_vecf(const std::filesystem::path& filename) -> Eigen::VectorXf
{
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs) {
        throw std::runtime_error("Failed to open file: " + filename.string());
    }

    const auto npy_data = npy::read_npy<float>(ifs);
    return to_eigen_vector(npy_data);
}

// ---------------------------------------------------------------------------------------------------------------------

auto npy_file_to_eigen_vecd(const std::filesystem::path& filename) -> Eigen::VectorXd
{
    std::ifstream ifs(filename, std::ios::binary);
    if (!ifs) {
        throw std::runtime_error("Failed to open file: " + filename.string());
    }

    const auto npy_data = npy::read_npy<double>(ifs);
    return to_eigen_vector(npy_data);
}

// ---------------------------------------------------------------------------------------------------------------------


// =====================================================================================================================

NAMESPACE_END(utils)
NAMESPACE_END(volanim)

// =====================================================================================================================
