// =====================================================================================================================
//
// Created by Fabian Kemper on 2025-04-10.
//
// =====================================================================================================================

#pragma once
#ifndef VOLUMETRIC_STATE_UTILS_H
#define VOLUMETRIC_STATE_UTILS_H

// =====================================================================================================================

#include <vector>

#include "volanim_utils/defs.h"
#include "volumetric_base/state/volumetric_state.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(utils)

// =====================================================================================================================

// get local matrices for animation rig in given volumetric state
[[nodiscard]]
auto local_rotation_matrix(const VolumetricState* state) -> Eigen::MatrixXd;

auto local_rotation_matrix(const VolumetricState* state, Eigen::MatrixXd & result) -> void;

// apply local rotation matrices to joint rotation from given state
auto pose_from_rotation_matrices(VolumetricState* state, const Eigen::MatrixXd& rotations) -> void;



struct StateModification
{
public:
    // setup state from given data directory
    static auto setup_state(VolumetricState* state, const std::filesystem::path& data_dir, Gender gender = Gender::MALE) -> void;

    // perform retargeting, for given layers (set_layers from volumetric state)
    static auto retarget_skeleton(VolumetricState* state) -> void;

    // set current pose to internal rig from loaded rodrigues rotation axis angle
    static auto set_pose(VolumetricState* state, const std::vector<float>& pose_rotvec) -> void;
    static auto set_translation(VolumetricState* state, const pmp::vec3& translation) -> void;

    // clear bindpose (use with caution!)
    static auto clear_bindpose(VolumetricState* state) -> void;

protected:
    // set new rbf warped high-res skeleton
    static auto calculate_new_highres_skeleton(VolumetricState* state) -> void;

    // set skinning parameters from character
    static auto set_skinning_parameters_from_character_for_layer(VolumetricState* state, Character* character, Layer layer, const std::string& mesh_name) -> void;

    // set animation skeleton (rig) from a given character
    static auto set_rig_from_character(VolumetricState* state, const Character* character) -> void;

    // set skinning weights to mesh
    static auto set_skinning_weights_to_mesh(const VolumetricState* state, pmp::SurfaceMesh& mesh, Layer layer) -> void;
    static auto set_skinning_weights_to_template_layers(VolumetricState* state) -> void;

    static auto set_bindpose(VolumetricState* state, const std::vector<float>& bindpose_rotvec) -> void;

    // helper methods for setting up volumetric state
    static auto read_highres_meshes(VolumetricState* state, Gender gender) -> void;
    static auto read_highres_wraps(VolumetricState* state, Gender gender) -> void;

    static auto read_mean_layers(VolumetricState* state, Gender gender) -> void;
    static auto read_template_layers(VolumetricState* state, Gender gender) -> void;

    static auto read_rig(VolumetricState* state, Gender gender) -> void;
    static auto read_collapsed_vertices(VolumetricState* state) -> void;

    static auto read_bindpose_from_file(VolumetricState* state, const std::filesystem::path& bindpose_file) -> void;
};





// =====================================================================================================================

NAMESPACE_END(utils)
NAMESPACE_END(volumetric_base)

// =====================================================================================================================

#endif //VOLUMETRIC_STATE_UTILS_H

// =====================================================================================================================
