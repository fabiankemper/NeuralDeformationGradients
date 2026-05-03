// =====================================================================================================================
// Created by fabian on 4/14/25.
// mail@fabiankemper.de
// =====================================================================================================================

#pragma once
#ifndef VOLUMETRIC_BASE_FILENAMES_H
#define VOLUMETRIC_BASE_FILENAMES_H

// =====================================================================================================================

#include <string_view>

#include "volumetric_base/volumetric_types.h"
#include "volumetric_base/state/configuration.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volanim)
NAMESPACE_BEGIN(filenames)

// =====================================================================================================================

using Gender = volumetric_base::Gender;
using Layer = volumetric_base::Layer;

// =====================================================================================================================


// ---------------------------------------------------------------------------------------------------------------------
// base directories

inline auto template_directory() -> std::filesystem::path
{
    return volanim::config::settings.data_dir / "template";
}

inline auto template_wrap_directory() -> std::filesystem::path
{
    return volanim::config::settings.data_dir / "template" / "wraps";
}

inline auto mean_directory() -> std::filesystem::path
{
    return volanim::config::settings.data_dir / "mean";
}

inline auto bary_warp_directory() -> std::filesystem::path
{
    return volanim::config::settings.data_dir / "bary_warp";
}

inline auto model_directory() -> std::filesystem::path
{
    return volanim::config::settings.data_dir / "models";
}

// ---------------------------------------------------------------------------------------------------------------------
// template files, with skinning weights

inline auto template_skel_hires_f() -> std::filesystem::path
{
    return template_directory() / "f_skeleton.fbx";
}

inline auto template_skel_wrap_f() -> std::filesystem::path
{
    // Note: This is the male version!! -> use only for skinning weights!
    return template_directory() / "m_skel_wrap.fbx";
}

inline auto template_musc_hires_f() -> std::filesystem::path
{
    return template_directory() / "f_musculature.fbx";
}

inline auto template_skin_f() -> std::filesystem::path
{
    return template_directory() / "f_skin.fbx";
}

inline auto template_skel_hires_m() -> std::filesystem::path
{
    return template_directory() / "m_skeleton.fbx";
}

inline auto template_skel_wrap_m() -> std::filesystem::path
{
    return template_directory() / "m_skel_wrap.fbx";
}

inline auto template_musc_hires_m() -> std::filesystem::path
{
    return template_directory() / "m_musculature.fbx";
}

inline auto template_skin_m() -> std::filesystem::path
{
    return template_directory() / "m_skin.fbx";
}

// ---------------------------------------------------------------------------------------------------------------------
// template files, layers (wraps)

inline auto template_layer_skel_f() -> std::filesystem::path
{
    return template_wrap_directory() / "template_f_skel_wrap.obj";
}

inline auto template_layer_musc_f() -> std::filesystem::path
{
    return template_wrap_directory() / "template_f_musc_wrap.obj";
}

inline auto template_layer_skin_f() -> std::filesystem::path
{
    return template_wrap_directory() / "template_f_skin.obj";
}

inline auto template_layer_skel_m() -> std::filesystem::path
{
    return template_wrap_directory() / "template_m_skel_wrap.obj";
}

inline auto template_layer_musc_m() -> std::filesystem::path
{
    return template_wrap_directory() / "template_m_musc_wrap.obj";
}

inline auto template_layer_skin_m() -> std::filesystem::path
{
    return template_wrap_directory() / "template_m_skin.obj";
}

// helper method
auto template_layer(const Layer& layer, const Gender& gender) -> std::filesystem::path;


// ---------------------------------------------------------------------------------------------------------------------
// rig

inline auto template_rig_m() -> std::filesystem::path
{
    return template_directory() / "male.rig";
}

inline auto template_rig_f() -> std::filesystem::path
{
    return template_directory() / "female.rig";
}

auto template_rig(const Gender& gender) -> std::filesystem::path;


// ---------------------------------------------------------------------------------------------------------------------
// mean layers

inline auto mean_layer_skel_f() -> std::filesystem::path
{
    return mean_directory() / "mean_female_skel.obj";
}

inline auto mean_layer_musc_f() -> std::filesystem::path
{
    return mean_directory() / "mean_female_musc.obj";
}

inline auto mean_layer_skin_f() -> std::filesystem::path
{
    return mean_directory() / "mean_female_skin.obj";
}

inline auto mean_layer_skel_m() -> std::filesystem::path
{
    return mean_directory() / "mean_male_skel.obj";
}

inline auto mean_layer_musc_m() -> std::filesystem::path
{
    return mean_directory() / "mean_male_musc.obj";
}

inline auto mean_layer_skin_m() -> std::filesystem::path
{
    return mean_directory() / "mean_male_skin.obj";
}

// helper
auto mean_layer(const Layer& layer, const Gender& gender) -> std::filesystem::path;

// ---------------------------------------------------------------------------------------------------------------------
// barycentric warp files

inline auto bary_steiner_nodes_f() -> std::filesystem::path
{
    return bary_warp_directory() / "skeltets_f.node";
}

inline auto bary_steiner_nodes_m() -> std::filesystem::path
{
    return bary_warp_directory() / "skeltets_m.node";
}

inline auto bary_steiner_tets_f() -> std::filesystem::path
{
    return bary_warp_directory() / "skeltets_f.ele";
}

inline auto bary_steiner_tets_m() -> std::filesystem::path
{
    return bary_warp_directory() / "skeltets_m.ele";
}

// Male file paths
auto inline bary_coeffs_f() -> std::filesystem::path
{
    return bary_warp_directory() / "female_coefficients.npy";
}

auto inline bary_coeffs_m() -> std::filesystem::path
{
    return bary_warp_directory() / "male_coefficients.npy";
}

auto inline bary_muscle_indices_f() -> std::filesystem::path
{
    return bary_warp_directory() / "female_muscle_indices.npy";
}

auto inline bary_muscle_indices_m() -> std::filesystem::path
{
    return bary_warp_directory() / "male_muscle_indices.npy";
}

auto inline bary_prism_indices_f() -> std::filesystem::path
{
    return bary_warp_directory() / "female_prism_indices.npy";
}

auto inline bary_prism_indices_m() -> std::filesystem::path
{
    return bary_warp_directory() / "male_prism_indices.npy";
}

auto inline bary_template_path_f() -> std::filesystem::path
{
    return bary_warp_directory() / "female_template_positions.npy";
}

auto inline bary_template_path_m() -> std::filesystem::path
{
    return bary_warp_directory() / "male_template_positions.npy";
}

// high level helper
auto bary_steiner_nodes(const Gender& gender) -> std::filesystem::path;
auto bary_steiner_tets(const Gender& gender) -> std::filesystem::path;
auto bary_coeffs(const Gender& gender) -> std::filesystem::path;
auto bary_template(const Gender& gender) -> std::filesystem::path;
auto bary_muscle_indices(const Gender& gender) -> std::filesystem::path;
auto bary_prism_indices(const Gender& gender) -> std::filesystem::path;


// ---------------------------------------------------------------------------------------------------------------------
// models
auto get_model_path(const std::string& model_filename) -> std::filesystem::path;

auto defgrad_pca_mean() -> std::filesystem::path;
auto defgrad_pca_weights() -> std::filesystem::path;
auto defgrad_onnx_model() -> std::filesystem::path;

auto vertex_onnx_model() -> std::filesystem::path;


// ---------------------------------------------------------------------------------------------------------------------
// fbx node names
inline constexpr std::string_view template_node_skeleton_hires = "skeleton";
inline constexpr std::string_view template_node_skeleton_wrap = "skel_wrap";
inline constexpr std::string_view template_node_musc_hires = "musculature";
inline constexpr std::string_view template_node_skin = "skin";

inline constexpr std::string_view mean_node_skel_wrap = "mean_skel_wrap";
inline constexpr std::string_view mean_node_musc_wrap = "mean_musc_wrap";
inline constexpr std::string_view mean_node_skin = "mean_skin";

// ---------------------------------------------------------------------------------------------------------------------
// miscellaneous

// collapsed vertices
inline auto collapsed_vertices() -> std::filesystem::path
{
    return template_directory() / "head_hands_toes.sel";
}

// bindpose
inline auto template_bindpose() -> std::filesystem::path
{
    return volanim::config::settings.data_dir / "animation" / "boerner_tpose_rotvec.npy";
}

// number of center for rbf war on high resolution skeleton
inline constexpr size_t rbf_centers_hires_skeleton = 100;

// =====================================================================================================================

NAMESPACE_END(filenames)
NAMESPACE_END(volanim)

// =====================================================================================================================

#endif //VOLUMETRIC_BASE_FILENAMES_H

// =====================================================================================================================
