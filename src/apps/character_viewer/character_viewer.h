// =====================================================================================================================

#pragma once
#ifndef PMPCHARACTER_VIEWER_H
#define PMPCHARACTER_VIEWER_H

// =====================================================================================================================

#include <filesystem>

#include <pmp/visualization/trackball_viewer.h>

#include <pmp_character/node_types/Character.h>

#include "volanim_utils/defs.h"
#include "volumetric_base/state/volumetric_state.h"
#include "volumetric_character/volumetric_character.h"
#include "volumetric_character/renderer/volumetric_character_renderer.h"

#include "character_viewer/animation/timeline.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volanim)

using Character = pmp_character::Character;
using SurfaceMesh = pmp::SurfaceMesh;

using Selection = utils::Selection;

using Layer = volumetric_character::Layer;
using Timeline = volumetric_character::animation::Timeline;
using VolumetricCharacter = volumetric_character::VolumetricCharacter;
using VolumetricData = volumetric_character::VolumetricState;
using VolumetricRenderer = volumetric_character::VolumetricCharacterRenderer;

using Path = std::filesystem::path;

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

// todo: find a better solution for this memory region
constexpr unsigned int N_MeshNames = 15;

// ---------------------------------------------------------------------------------------------------------------------

class CharacterViewer : public pmp::TrackballViewer
{
public:
    explicit CharacterViewer();
    CharacterViewer(CharacterViewer& other); // copy-constructor
    CharacterViewer& operator=(const CharacterViewer& other); // copy-assignment
    ~CharacterViewer() override;

    // prepare everything
    auto setup_state(const Path& data_dir, volumetric_base::Gender gender) -> void;

    // todo: remove this
    [[nodiscard]]
    auto read_all_meshes_for_sample(int sample_num) const -> volumetric_base::LayeredMesh;

    auto read_animation(
        const Path& animation_filename
    ) -> void;

    auto set_samples_dir(
        const Path& samples_dir
    ) -> void;

protected:
    // volumetric character and renderer pointer
    VolumetricCharacter* _vol_character = nullptr;
    VolumetricRenderer* _vol_renderer = nullptr;

    // current loaded timeline
    Timeline* _timeline = nullptr;

    // todo: remove (see above)
    std::vector<pmp::SurfaceMesh> _meshes;

    // --------- GUI --------
    // alpha
    std::array<float, 3> _layer_alpha = {1.0F, 1.0F, 0.76F};
    bool _render_skeleton = false;
    bool render_self_intersections = false;
    int _active_mesh_index = -1;
    int _active_renderer_index = -1; // current renderer pointer
    bool _converge = false;

    // pose
    std::vector<float> _joint_angles;
    std::array<float, 3> _translation = {0, 0, 0};
    bool _no_pelvis_rotation = false;
    bool _no_extremities_rotation = true;

    // deformation transfer
    int _inverse_sample_id = 4001;
    int mesh_id = 0;

    // valid sample numbers
    std::vector<std::string> _sample_numbers;
    int _skinning_mode = 0;

    // gui tools
    bool _draw_weights = false;

    // render call
    auto draw(const std::string& draw_mode) -> void override;
    auto do_processing() -> void override;

    // keyboard press
    auto keyboard(int key, int code, int action, int mods) -> void override;

    // imgui
    auto process_imgui() -> void override;
    auto process_imgui_view() -> void;
    auto process_imgui_body_shape() -> void;
    auto process_imgui_animation() -> void;
    auto process_imgui_skinning() -> void;
    auto process_imgui_pose() -> void;
    auto process_imgui_export() -> void;
    auto process_imgui_widgets() const -> void;

    auto drop(int count, const char** paths) -> void override;

    // re-create renderer
    auto update_renderer() -> void;

    // update meshes
    auto update_layers() -> void;
    auto update_use_template(bool use_template) -> void;
    auto update_timeline() -> void;
    auto update_pose() const -> void;

    // clean pose according to settings
    auto clean_pose_trans(std::vector<float> pose, std::array<float, 3> trans) const -> std::tuple<std::vector<float>, std::array<float, 3>>;

};

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(volanim)

// =====================================================================================================================

#endif //PMPCHARACTER_VIEWER_H

// =====================================================================================================================
