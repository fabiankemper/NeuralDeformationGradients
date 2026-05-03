// =====================================================================================================================
//
// Created by Fabian Kemper on 2025-08-06.
//
// =====================================================================================================================

#include "filenames.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volanim)
NAMESPACE_BEGIN(filenames)

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

auto template_layer(const Layer& layer, const Gender& gender) -> std::filesystem::path
{
    switch (gender) {
        case Gender::FEMALE:
            switch (layer) {
                case Layer::SKEL:
                    return template_layer_skel_f();
                    break;
                case Layer::MUSC:
                    return template_layer_musc_f();
                    break;
                case Layer::SKIN:
                    return template_layer_skin_f();
                    break;
                default:
                    throw std::runtime_error("TemplateLayer: Invalid layer");
            }
            break;
        case Gender::MALE:
            switch (layer) {
                case Layer::SKEL:
                    return template_layer_skel_m();
                    break;
                case Layer::MUSC:
                    return template_layer_musc_m();
                    break;
                case Layer::SKIN:
                    return template_layer_skin_m();
                    break;
                default:
                    throw std::runtime_error("TemplateLayer: Invalid layer");
            }
        default:
            throw std::runtime_error("TemplateLayer: Invalid gender");
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto template_rig(const Gender& gender) -> std::filesystem::path
{
    switch (gender) {
        case Gender::FEMALE:
            return template_rig_f();
            break;
        case Gender::MALE:
            return template_rig_m();
            break;
        default:
            throw std::runtime_error("TemplateRig: Invalid gender");
    }
}

auto mean_layer(const Layer& layer, const Gender& gender) -> std::filesystem::path
{
    switch (gender) {
        case Gender::FEMALE:
            switch (layer) {
            case Layer::SKEL:
                    return mean_layer_skel_f();
                    break;
            case Layer::MUSC:
                    return mean_layer_musc_f();
                    break;
            case Layer::SKIN:
                    return mean_layer_skin_f();
                    break;
            default:
                    throw std::runtime_error("MeanLayer: Invalid layer");
            }
            break;
        case Gender::MALE:
            switch (layer) {
            case Layer::SKEL:
                    return mean_layer_skel_m();
                    break;
            case Layer::MUSC:
                    return mean_layer_musc_m();
                    break;
            case Layer::SKIN:
                    return mean_layer_skin_m();
                    break;
            default:
                    throw std::runtime_error("MeanLayer: Invalid layer");
            }
        default:
            throw std::runtime_error("MeanLayer: Invalid gender");
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto bary_steiner_nodes(const Gender& gender) -> std::filesystem::path
{
    switch (gender) {
        case Gender::FEMALE:
            return bary_steiner_nodes_f();
            break;
        case Gender::MALE:
            return bary_steiner_nodes_m();
            break;
        default:
            throw std::runtime_error("Bary Steiner Nodes: Invalid gender");
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto bary_steiner_tets(const Gender& gender) -> std::filesystem::path
{
    switch (gender) {
        case Gender::FEMALE:
            return bary_steiner_tets_f();
            break;
        case Gender::MALE:
            return bary_steiner_tets_m();
            break;
        default:
            throw std::runtime_error("Bary Steiner Nodes: Invalid gender");
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto bary_coeffs(const Gender& gender) -> std::filesystem::path
{
    switch (gender) {
        case Gender::FEMALE:
            return bary_coeffs_f();
            break;
        case Gender::MALE:
            return bary_coeffs_m();
            break;
        default:
            throw std::runtime_error("Bary Steiner Nodes: Invalid gender");
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto bary_template(const Gender& gender) -> std::filesystem::path
{
    switch (gender) {
        case Gender::FEMALE:
            return bary_template_path_f();
            break;
        case Gender::MALE:
            return bary_template_path_m();
            break;
        default:
            throw std::runtime_error("Bary Steiner Nodes: Invalid gender");
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto bary_muscle_indices(const Gender& gender) -> std::filesystem::path
{
    switch (gender) {
        case Gender::FEMALE:
            return bary_muscle_indices_f();
            break;
        case Gender::MALE:
            return bary_muscle_indices_m();
            break;
        default:
            throw std::runtime_error("Bary Steiner Nodes: Invalid gender");
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto bary_prism_indices(const Gender& gender) -> std::filesystem::path
{
    switch (gender) {
        case Gender::FEMALE:
            return bary_prism_indices_f();
            break;
        case Gender::MALE:
            return bary_prism_indices_m();
            break;
        default:
            throw std::runtime_error("Bary Steiner Nodes: Invalid gender");
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto get_model_path(const std::string& model_filename) -> std::filesystem::path
{
    return model_directory() / model_filename;
}

// ---------------------------------------------------------------------------------------------------------------------

auto defgrad_pca_mean() -> std::filesystem::path
{
    return model_directory() / "defgrad_pca_mean.npy";
}

// ---------------------------------------------------------------------------------------------------------------------

auto defgrad_pca_weights() -> std::filesystem::path
{
    return model_directory() / "defgrad_pca_matrix.npy";
}

// ---------------------------------------------------------------------------------------------------------------------

auto defgrad_onnx_model() -> std::filesystem::path
{
    return model_directory() / "defgrad.onnx";
}

// ---------------------------------------------------------------------------------------------------------------------

auto vertex_onnx_model() -> std::filesystem::path
{
    return model_directory() / "vertex.onnx";
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(filenames)

NAMESPACE_END(volanim)

// =====================================================================================================================

