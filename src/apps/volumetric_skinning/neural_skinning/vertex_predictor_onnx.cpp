// --------------------------------------------------------

#include "vertex_predictor_onnx.h"

#ifdef __APPLE__
#include <coreml_provider_factory.h>
#endif

#include "volanim_utils/npy_utils.h"
#include "volumetric_base/state/filenames.h"

// --------------------------------------------------------
namespace volumetric_skinning::onnx {
VertexPredictor::VertexPredictor(const std::filesystem::path& model_path)
    : env(ORT_LOGGING_LEVEL_WARNING, "VertexPredictor"), session(),
      cpu_memory_info(
          Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)
      ),
      input_tensor_(), output_tensor_()
{
    // Load data (Eigen handles the conversion to RowMajor automatically here)
    pca_matrix = volanim::utils::npy_file_to_eigen_matf(
        volanim::filenames::defgrad_pca_weights()
    );
    std::cout << "Loaded PCA Matrix " << pca_matrix.rows() << "x"
        << pca_matrix.cols() << std::endl;
    pca_mean = volanim::utils::npy_file_to_eigen_vecf(
        volanim::filenames::defgrad_pca_mean()
    );


    try {
        auto available_providers = Ort::GetAvailableProviders();

        auto is_available = [&](const std::string& name) {
            return std::ranges::find(available_providers, name) != available_providers.end();
        };

        // try cuda
        if (is_available("CUDAExecutionProvider")) {
            OrtCUDAProviderOptions cuda_options {};
            session_options.AppendExecutionProvider_CUDA(cuda_options);
            std::cout << "[VertexPredictor] Using CUDA acceleration.\n";
        }
#ifdef __APPLE__
        // note: disabled coreML for now, as it is not working
        else if (is_available("CoreMLExecutionProvider") and false) {
            const uint32_t coreml_flags = 0;
            // Use the C-API function call which works with the C++ session_options object
            const OrtStatus* status = OrtSessionOptionsAppendExecutionProvider_CoreML(session_options, coreml_flags);
            if (status == nullptr) {
                std::cout << "[VertexPredictor] Using CoreML Acceleration.\n";
            } else {
                std::cerr << "[VertexPredictor] CoreML setup failed.\n";
            }
        }
#endif
        else {
            // fallback to cpu
            std::cout << "[VertexPredictor] Using no acceleration.\n";
        }

        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
#ifdef WIN32
        session = std::make_unique<Ort::Session>(env, model_path.c_str(), session_options);
#else
        // On macOS/Linux, convert wstring to string
        const std::string path_str = model_path.string();
        session = std::make_unique<Ort::Session>(env, path_str.c_str(), session_options);
#endif

    } catch (const std::exception& e) {
        std::cerr << "[VertexPredictor] Initialization Error: " << e.what() << std::endl;
    }
}

// --------------------------------------------------------

void VertexPredictor::setup_optimized_io(size_t total_input_size)
{
    input_shape = {1, static_cast<int64_t>(total_input_size)};

    // Dummy tensor to discover output shape
    std::vector<float> dummy_in(total_input_size, 0.0f);
    Ort::Value const temp_in = Ort::Value::CreateTensor<float>(
        cpu_memory_info, dummy_in.data(), dummy_in.size(), input_shape.data(),
        input_shape.size()
    );

    const auto initial_output = session->Run(
        Ort::RunOptions{nullptr}, input_names, &temp_in, 1, output_names, 1
    );

    size_t const output_size =
        initial_output.front().GetTensorTypeAndShapeInfo().GetElementCount();
    output_shape = initial_output.front().GetTensorTypeAndShapeInfo().GetShape();

    input_buffer.resize(total_input_size, 0.0F);
    output_buffer.resize(output_size, 0.0F);
    final_correctives.resize(static_cast<int64_t>(output_size / 3), 3);

    input_tensor_ = std::make_unique<Ort::Value>(Ort::Value::CreateTensor<float>(
            cpu_memory_info, input_buffer.data(), input_buffer.size(),
            input_shape.data(), input_shape.size()
        )
    );

    output_tensor_ = std::make_unique<Ort::Value>(Ort::Value::CreateTensor<float>(
            cpu_memory_info, output_buffer.data(), output_buffer.size(),
            output_shape.data(), output_shape.size()
        )
    );

    is_optimized_ = true;
    std::cout << "[VertexPredictor] Preallocated zero-copy arrays. Input: "
        << total_input_size << " | Output: " << output_size << std::endl;
}

// --------------------------------------------------------

void VertexPredictor::set_body(const Eigen::VectorXf& body)
{
    if (body_diff_buffer.size() != body.size()) {
        body_diff_buffer.resize(body.size());
    }
    body_diff_buffer.noalias() = body - pca_mean;
    cached_pca_result.noalias() = pca_matrix * body_diff_buffer;

    body_is_set_ = true;
}

// --------------------------------------------------------

auto VertexPredictor::predict(const Eigen::VectorXf& poses) -> void
{
    if (!body_is_set_) {
        throw std::runtime_error(
            "[VertexPredictor] You must call set_body() before predict()!"
        );
    }

    const size_t pca_dim = cached_pca_result.size();
    const size_t pose_dim = poses.size();
    const size_t in_dim = pca_dim + pose_dim;

    if (!is_optimized_ || input_buffer.size() != in_dim) {
        setup_optimized_io(in_dim);
    }

    // Map buffer and cleanly copy the cached body and new poses
    Eigen::Map<Eigen::VectorXf> in_map(input_buffer.data(), in_dim);
    in_map.head(pca_dim) = cached_pca_result;
    in_map.tail(pose_dim) = poses;

    session->Run(Ort::RunOptions{nullptr}, input_names, input_tensor_.get(), 1,
                 output_names, output_tensor_.get(), 1
    );

    Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>>
        raw_result(output_buffer.data(), output_buffer.size() / 3, 3);

    const int64_t num_matrices = raw_result.rows() / 3;

#pragma omp parallel for schedule(static, 64)
    for (int64_t i = 0; i < num_matrices; ++i) {
        Eigen::Matrix3f const block = raw_result.block<3, 3>(i * 3, 0);
        final_correctives.block<3, 3>(i * 3, 0) = block.cast<double>();
    }
}
} // namespace volumetric_skinning::onnx
// --------------------------------------------------------
