// --------------------------------------------------------

#pragma once

// --------------------------------------------------------

#include <Eigen/Core>
#include <vector>
#include <memory>

#include <onnxruntime_cxx_api.h>

#include "../base/io.hpp"

// --------------------------------------------------------

class DefGradPredictor
{
protected:

public:
    // Typedef for our cache-friendly RowMajor output matrix
    using DefGradMatrix = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;
    // Optional: Make PCA matrix RowMajor to speed up set_body() if you swap characters often
    using PCAMatrix = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    Ort::Env env;
    Ort::SessionOptions session_options;
    std::unique_ptr<Ort::Session> session;
    Ort::MemoryInfo cpu_memory_info;

    const char* input_names[1] = {"input"};
    const char* output_names[1] = {"output"};

    PCAMatrix pca_matrix;
    Eigen::VectorXf pca_mean;

    // --- State and Caching ---
    Eigen::VectorXf body_diff_buffer;
    Eigen::VectorXf cached_pca_result;
    bool body_is_set_ = false;
    bool is_optimized_ = false;

    // --- Preallocated data buffers ---
    std::vector<float> input_buffer;
    std::vector<float> output_buffer;
    std::vector<int64_t> input_shape;
    std::vector<int64_t> output_shape;

    std::unique_ptr<Ort::Value> input_tensor_;
    std::unique_ptr<Ort::Value> output_tensor_;

    DefGradMatrix final_defgrads;

    explicit DefGradPredictor(const std::filesystem::path& model_path);

    auto setup_optimized_io(size_t total_input_size) -> void;

    auto set_body(const Eigen::VectorXf& body) -> void;

    auto predict(const Eigen::VectorXf& poses) -> void;
};
