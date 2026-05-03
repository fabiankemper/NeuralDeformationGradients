// =====================================================================================================================

#include "character_viewer.h"

#include <regex>
#include <tuple>
#include <unordered_set>

#include <imgui.h>

#include <pmp/io/io.h>
#include <pmp_character/fbxfile/fbx_export.h>
#include <pmp_character/fbxfile/fbx_file.h>

#include "volanim_utils/random_utils.h"
#include "volanim_utils/string_utils.h"
#include "volumetric_base/state/io.h"
#include "volumetric_base/state/skinning/skinning.h"
#include "volumetric_base/state/utils.h"
#include "volumetric_base/utility/imgui_util.h"
#include "volumetric_skinning/base/volumetric_boundary_fixer.h"
#include "volumetric_skinning/neural_skinning/vertex_skinner.h"
#include "volumetric_skinning/stable_neo_hookean_elasticity/StableNeoHookeanSkinner.h"

#include "character_viewer/constants.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volanim)
// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

CharacterViewer::CharacterViewer()
    : pmp::TrackballViewer("Character Viewer", 1280, 720)
{
    clear_draw_modes();
    //    add_draw_mode("Texture");
    add_draw_mode("Smooth Shading");
    add_draw_mode("Hidden Line");
    set_draw_mode("Smooth Shading");

    _vol_character = new VolumetricCharacter();
    _vol_character->set_skinning_mode(static_cast<VolumetricCharacter::SkinningMode>(_skinning_mode));
}

// ---------------------------------------------------------------------------------------------------------------------

CharacterViewer::CharacterViewer(CharacterViewer& other)
    : pmp::TrackballViewer("Copy", other.width(), other.height())
{
    std::cerr << "Copy constructor is not implemented" << std::endl;
}

// ---------------------------------------------------------------------------------------------------------------------

CharacterViewer& CharacterViewer::operator=(const CharacterViewer& other)
{
    if (this == &other) {
        return *this;
    }

    _vol_character = other._vol_character;
    _vol_renderer = other._vol_renderer;

    return *this;
}

// ---------------------------------------------------------------------------------------------------------------------

CharacterViewer::~CharacterViewer()
{
    delete _vol_character;
    delete _vol_renderer;
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::setup_state(
    const Path& data_dir,
    volumetric_base::Gender gender
) -> void
{
    volumetric_base::utils::StateModification::setup_state(_vol_character->state(), data_dir, gender);

    // bigger radius for animations
    const auto& rig = _vol_character->state()->rig;
    set_scene(rig.get_center(), 5.0f * rig.get_size());
    translate(pmp::vec3{0, 0, 25.0}); // move camera closer to character

    // get pose parameters, each joint has 3 rotation axis
    _joint_angles.resize(3 * rig.joints_.size());

    // update and initialize everything for animation
    _vol_character->updated_template_layers(); // re-create skinnable mesh
    _vol_character->updated_mean_meshes();

    // re-create renderer (after loaded state)
    update_renderer();
    _vol_character->set_dirty(true);
    _vol_character->set_skeleton_dirty(true);
}

// ---------------------------------------------------------------------------------------------------------------------s

auto CharacterViewer::read_all_meshes_for_sample(const int sample_num) const
    -> volumetric_base::LayeredMesh
{
    const auto& config = config::settings;

    SurfaceMesh skel, musc, skin;

    const std::string skel_name = _sample_numbers[sample_num] + "_" + SAMPLES_SKELS + SAMPLES_FILE_EXT;
    const std::string musc_name = _sample_numbers[sample_num] + "_" + SAMPLES_MUSCS + SAMPLES_FILE_EXT;
    const std::string skin_name = _sample_numbers[sample_num] + "_" + SAMPLES_SKINS + SAMPLES_FILE_EXT;

    pmp::read(skel, config.samples_dir / SAMPLES_SKELS / skel_name);
    pmp::read(musc, config.samples_dir / SAMPLES_MUSCS / musc_name);
    pmp::read(skin, config.samples_dir / SAMPLES_SKINS / skin_name);

    return {skel, musc, skin};
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::read_animation(const Path& animation_filename) -> void
{
    // delete _timeline;
    if (_timeline == nullptr) {
        _timeline = new Timeline();
    }

    if (animation_filename.extension() == ".npy") {
        _timeline->from_numpy(animation_filename);
    }
    if (animation_filename.extension() == ".npz") {
        _timeline->from_numpyz(animation_filename);
    }
    if (animation_filename.extension() == ".csv") {
        _timeline->from_csv(animation_filename);
    }
    if (animation_filename.extension() == ".pose") {
        _timeline->from_posefile(animation_filename);
    }
    update_pose();
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::set_samples_dir(const Path& samples_dir) -> void
{
    // read all numbers in skeleton dir, then check if all numbers exists in musc
    // and skin directory
    if (exists(samples_dir)) {
        for (const auto& entry: std::filesystem::directory_iterator(samples_dir / SAMPLES_SKELS)) {
            auto filename = Path(entry).filename();

            // extract numbers from filename
            std::string numbers = std::regex_replace(
                filename.string(), std::regex("[^0-9]*([0-9]+).*"), std::string("$1"));
            _sample_numbers.emplace_back(numbers);
        }
    }

    // sort numbers
    std::ranges::sort(_sample_numbers);

    // check musc and skin existence
    std::erase_if(_sample_numbers, [&](const std::string& entry) -> bool {
        const std::string musc_fn = entry + "_" + SAMPLES_MUSCS + SAMPLES_FILE_EXT;
        const std::string skin_fn = entry + "_" + SAMPLES_SKINS + SAMPLES_FILE_EXT;
        const auto musc_filename = samples_dir / SAMPLES_MUSCS / musc_fn;
        const auto skin_filename = samples_dir / SAMPLES_SKINS / skin_fn;
        return not exists(musc_filename) or not exists(skin_filename);
    });

    std::cout << "Found " << _sample_numbers.size() << " body samples." << std::endl;
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::draw(const std::string& draw_mode) -> void
{
    if (_vol_renderer && _active_mesh_index < 0) {
        // check new frame in timeline
        update_timeline();

        // compute required matrices
        const pmp::mat4 current_modelview = modelview_matrix_;
        const pmp::mat3 normal_matrix = inverse(transpose(linear_part(current_modelview)));
        const pmp::mat4 model_view_projection = projection_matrix_ * current_modelview;

        // render step
        _vol_renderer->draw(current_modelview, model_view_projection, normal_matrix, draw_mode);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void CharacterViewer::do_processing()
{
    TrackballViewer::do_processing();

    if (_timeline && _timeline->playing()) {
        _timeline->tick();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void CharacterViewer::keyboard(int key, int code, int action, int mods)
{
    TrackballViewer::keyboard(key, code, action, mods);

    const volumetric_base::VolumetricState* state = nullptr;
    if (_vol_character != nullptr) {
        state = _vol_character->state();
    }

    if (action == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_BACKSPACE:
                // reset camera
                if (state != nullptr) {
                    modelview_matrix_ = pmp::mat4::identity();
                    set_scene(state->rig.get_center(), 5.0f * state->rig.get_size());
                    translate(pmp::vec3{0, 0, 25.0}); // move camera closer to character
                }
                break;
            default:
                break;
        }
    }
}


// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::process_imgui() -> void
{
    Window::process_imgui();

    process_imgui_view();
    process_imgui_body_shape();
    process_imgui_pose();
    process_imgui_animation();
    process_imgui_skinning();
    process_imgui_export();
    process_imgui_widgets();
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::process_imgui_view() -> void
{
    if (_vol_character->_skinner->dirty) {
        _vol_character->skinning();
        _vol_character->_skinner->dirty = false;
    }

    if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) {
        bool alpha_changed = false;

        ImGui::Text("Layer alpha");
        ImGui::PushItemWidth(100.0F);
        if (ImGui::SliderFloat("Skeleton", &_layer_alpha[0], 0.0F, 1.0F)) {
            alpha_changed = true;
        }
        if (ImGui::SliderFloat("Musculature", &_layer_alpha[1], 0.0F, 1.0F)) {
            alpha_changed = true;
        }
        if (ImGui::SliderFloat("Skin", &_layer_alpha[2], 0.0F, 1.0F)) {
            alpha_changed = true;
        }
        ImGui::PopItemWidth();

        if (ImGui::Button("Reset##alpha")) {
            _layer_alpha[0] = 1.0F;
            _layer_alpha[1] = 1.0F;
            _layer_alpha[2] = 0.76F;
            alpha_changed = true;

            if (_vol_character != nullptr) {
                _vol_character->highres_skinning = true;
                _vol_character->skinning();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Skin only")) {
            _layer_alpha[0] = 0.0F;
            _layer_alpha[1] = 0.0F;
            _layer_alpha[2] = 1.0F;
            alpha_changed = true;

            if (_vol_character != nullptr) {
                _vol_character->highres_skinning = false;
                _vol_character->skinning();
            }
        }

        if (alpha_changed && _vol_renderer != nullptr) {
            _vol_renderer->set_alpha(_layer_alpha[0], _layer_alpha[1],_layer_alpha[2]);
        }

        ImGui::Spacing();
        if (ImGui::Checkbox("Draw rig", &_render_skeleton) && _vol_renderer) {
            _vol_renderer->show_skeleton = _render_skeleton;
        }
        if (_vol_character != nullptr) {
            if (ImGui::Checkbox("HighRes Anatomy", &_vol_character->highres_skinning)) {
                _vol_character->skinning();
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::process_imgui_body_shape() -> void
{
    if (ImGui::CollapsingHeader("Body shape", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushItemWidth(150.0F);
        ImGui::InputInt("Mesh ID", &mesh_id);
        ImGui::PopItemWidth();

        if (ImGui::Button("Load##simple")) {
            update_use_template(false);
            update_layers();
        }

        ImGui::SameLine();
        if (ImGui::Button("Random##load_random") && not _sample_numbers.empty()) {
            const int sample_id = utils::get_random_int(_sample_numbers.size());
            mesh_id = sample_id;

            // update gui number
            if (sample_id >= 0 &&
                sample_id < static_cast<int>(_sample_numbers.size())) {
                _inverse_sample_id = std::stoi(_sample_numbers[sample_id]);
            }

            update_use_template(false);
            update_layers();
        }

        /* caesar dataset loading */
        ImGui::Spacing();
        ImGui::PushItemWidth(150.0F);
        ImGui::InputInt("Caesar ID", &_inverse_sample_id);
        if (ImGui::Button("Load##caesar")) {
            // search for index in sample numbers
            auto iter = std::ranges::find(_sample_numbers,std::to_string(_inverse_sample_id));
            if (iter != _sample_numbers.end()) {
                const auto current_index = static_cast<int>(std::distance(_sample_numbers.begin(), iter));
                mesh_id = current_index;
                update_use_template(false);
                update_layers();
            } else {
                std::cout << "Could not inverse find CaesarID=" << _inverse_sample_id << std::endl;
            }
        }
        ImGui::PopItemWidth();

        ImGui::Spacing();
        if (ImGui::Checkbox("Use template", &_vol_character->use_template_mesh)) {
            update_use_template(_vol_character->use_template_mesh);
        }

        /* TODO: PCA PARAMETERS */
        // ImGui::Spacing();
        // if (ImGui::CollapsingHeader("PCA parameters", ImGuiTreeNodeFlags_None)) {
        //     ImGui::Text("TODO.");
        // }
        ImGui::Spacing();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::process_imgui_animation() -> void
{
    if (ImGui::CollapsingHeader("Animation", ImGuiTreeNodeFlags_DefaultOpen)) {
        const std::string fps_label = std::to_string(static_cast<int>(_vol_character->current_fps)) + " FPS";
        ImGui::PushItemWidth(120.0F);
        ImGui::SetWindowFontScale(1.5f);
        ImGui::LabelText(fps_label.c_str(), "Skinning");
        ImGui::SetWindowFontScale(1.0f);
        ImGui::PopItemWidth();

        ImGui::Spacing();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::process_imgui_skinning() -> void
{
    if (_vol_character && ImGui::CollapsingHeader("Skinning Mode", ImGuiTreeNodeFlags_DefaultOpen)) {
        bool skinning_changed = false;

        if (ImGui::RadioButton("LBS", &_skinning_mode,
                static_cast<std::underlying_type_t<Layer>>( VolumetricCharacter::SkinningMode::LINEAR_BLEND_SKINNING))
            || ImGui::RadioButton("DQS", &_skinning_mode,
                static_cast<std::underlying_type_t<Layer>>(VolumetricCharacter::SkinningMode::DUAL_QUATERNION_SKINNING))
            || ImGui::RadioButton("Corot. Linear FEM", &_skinning_mode,
                static_cast<std::underlying_type_t<Layer>>(VolumetricCharacter::SkinningMode::ELASTIC_SKINNING))
            || ImGui::RadioButton("Projective Skinning", &_skinning_mode,
                static_cast<std::underlying_type_t<Layer>>(VolumetricCharacter::SkinningMode::PROJECTIVE_SKINNING))
            || ImGui::RadioButton("NDG", &_skinning_mode,
                static_cast<std::underlying_type_t<Layer>>(VolumetricCharacter::SkinningMode::NN_DEFGRAD_SKINNING))
            || ImGui::RadioButton("VertexNet", &_skinning_mode,
                static_cast<std::underlying_type_t<Layer>>(VolumetricCharacter::SkinningMode::NN_VERTEX_SKINNING))
            || ImGui::RadioButton("FEM (Neo-Hookean)", &_skinning_mode,
                static_cast<std::underlying_type_t<Layer>>(VolumetricCharacter::SkinningMode::NEO_HOOKEAN)))
        {
            _vol_character->set_skinning_mode(static_cast<VolumetricCharacter::SkinningMode>(_skinning_mode));
            _vol_character->update_layers();
            skinning_changed = true;
        }

        // plugin for skinner
        ImGui::Spacing();
        _vol_character->imgui_skinner();
        if (this->_vol_character->state()->get_dirty()) {
            volumetric_base::LayeredMesh combined = this->_vol_character->state()->_skinning_result;
            volumetric_skinning::VolumetricBoundaryFixer const fixer(combined);
            auto skel = _vol_character->skinnable_layer(Layer::SKEL);
            volumetric_base::skinning::lbs(_vol_character->state(), skel, volumetric_base::Layer::SKIN);

            pmp::SurfaceMesh new_skel = skel.bake_mesh();
            pmp::SurfaceMesh new_musc = skel.bake_mesh();
            pmp::SurfaceMesh new_skin = skel.bake_mesh();

            prism_adapter::transfer_to_surface_mesh(new_skel, combined, volumetric_base::Layer::SKEL);
            prism_adapter::transfer_to_surface_mesh(new_musc, combined, volumetric_base::Layer::MUSC);
            prism_adapter::transfer_to_surface_mesh(new_skin, combined, volumetric_base::Layer::SKIN);

            _vol_character->skinnable_layer(Layer::SKEL).skinning_from_baked_mesh(new_skel);
            _vol_character->skinnable_layer(Layer::MUSC).skinning_from_baked_mesh(new_musc);
            _vol_character->skinnable_layer(Layer::SKIN).skinning_from_baked_mesh(new_skin);

            _vol_character->state()->set_dirty(false);
            _vol_character->set_dirty();
        }

        ImGui::Spacing();

        // update skinner
        if (skinning_changed) {
            update_pose();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::process_imgui_pose() -> void
{
    if (_vol_character != nullptr &&
        ImGui::CollapsingHeader("Pose", ImGuiTreeNodeFlags_None)) {
        bool angles_changed = false;
        const auto& joints = _vol_character->state()->rig.joints_;

        // converge animation?
        ImGui::Spacing();
        if (ImGui::Checkbox("Simulation auto converge", &_converge)) {
            angles_changed = true;
        }
        imgui_util::HelpMarker(
            "If toggled, all iterative animation methods (Projective/Neo-Hookean "
            "FEM) will automatically iterate until convergence upon selection. Can "
            "take a while per frame.");

        // neutral pelivs?
        ImGui::Spacing();
        if (ImGui::Checkbox("Disable Global Rotation", &_no_pelvis_rotation)) {
            angles_changed = true;
        }

        if (ImGui::Checkbox("Disable Hand/Head/Toe rotation", &_no_extremities_rotation)) {
            angles_changed = true;
        }

        if (ImGui::Button("Reset joint angles")) {
            for (auto& joint_angle: _joint_angles) {
                joint_angle = 0.0F;
            }
            angles_changed = true;
        }

        // pose angle sliders
        ImGui::Spacing();
        ImGui::Text("Translation");
        if (ImGui::SliderFloat3("Translation", &_translation[0], -3.0F, 3.0F)) {
            angles_changed = true;
        }

        ImGui::Text("Joint angles");
        auto gui_joint_sliders = [](const std::string& name, float angles[]) -> bool {
            return ImGui::SliderFloat3(name.c_str(), angles, -M_PI, M_PI, "%.4f");
        };

        for (size_t index = 0; index < joints.size(); ++index) {
            const auto& joint = joints[index];
            angles_changed |=
                    gui_joint_sliders(joint->name_, &_joint_angles[index * 3]);
        }

        if (angles_changed) {
            update_pose();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::process_imgui_export() -> void
{
    if (ImGui::CollapsingHeader("Export")) {
        if (_vol_character == nullptr) {
            return;
        }

        const auto out_dir = config::current_work_dir / "out";
        if (not exists(out_dir)) {
            std::filesystem::create_directory(out_dir);
        }

        if (ImGui::Button("Export Current Meshes")) {
            volumetric_character::VolIOFlags io_flags;
            if (not _vol_character->highres_skinning) {
                io_flags.musc_wrap = true;
                io_flags.skel_wrap = true;
            }
            _vol_character->save_meshes(std::filesystem::current_path() / "out", "", io_flags);
        }
        imgui_util::HelpMarker("Saves Meshes in the Folder /out/. Layer Wraps and Hi-Res Anatomy are saved.");
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::process_imgui_widgets() const -> void
{
    if (_timeline != nullptr) {
        ImGui::SetNextWindowPos(ImVec2(256.0f, 4.0f), ImGuiCond_Once);
        ImGui::SetNextWindowSize(ImVec2(600.0f, 102.0f), ImGuiCond_Once);
        _timeline->process_imgui();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::drop(int count, const char** paths) -> void
{
    TrackballViewer::drop(count, paths);

    for (auto idx = 0; idx < count; ++idx) {
        // convert filename to path
        auto filename = Path(paths[idx]);
        auto extension = utils::string_to_lower(filename.extension().string());
        if (extension == ".npz" || extension == ".npy" || extension == ".pose" || extension == ".csv") {
            read_animation(filename);
            return;
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::update_renderer() -> void
{
    delete _vol_renderer;
    _vol_renderer = new VolumetricRenderer(*_vol_character);
    _vol_renderer->set_highres_weights(_vol_character->_hires_skinner);

    _vol_renderer->set_alpha(_layer_alpha[0], _layer_alpha[1], _layer_alpha[2]);
    _vol_renderer->show_skeleton = _render_skeleton;
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::update_layers() -> void
{
    // check ranges
    if (mesh_id < 0 || mesh_id >= static_cast<int>(_sample_numbers.size())) {
        std::cerr << "Index out of range for templates.\n";
        return;
    }

    std::cout << "Loaded mesh " << mesh_id << "\n";

    // update ALL layers (for prism volumetric deformation transfer)
    const auto layers_skel = read_all_meshes_for_sample(mesh_id);

    _vol_character->state()->set_layers(layers_skel);
    _vol_character->update_layers();

    // perform skinning
    update_pose();
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::update_use_template(const bool use_template) -> void
{
    _vol_character->use_template_mesh = use_template;

    update_layers();
    _vol_character->update_use_mean_mesh();
    update_pose();
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::update_timeline() -> void
{
    if (_timeline && _timeline->is_dirty()) {
        const auto pose = _timeline->current_pose();
        _translation = _timeline->current_trans();

        if (pose.size() == _joint_angles.size()) {
            // move buffer
            for (size_t index = 0; index < _joint_angles.size(); ++index) {
                // _joint_angles[index] = pose[index] * 180.0 / M_PI;
                _joint_angles[index] = pose[index];
            }
            update_pose();
        } else {
            std::cerr << "Joint angles count from animation do not match internal animation skeleton joint count.\n";
            std::cerr << "Joint angles n=" << _joint_angles.size() << " pose.size()=" << pose.size() << "\n";
        }
        _timeline->clear_dirty();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::update_pose() const -> void
{
    std::vector<float> joint_rotvecs(_joint_angles);
    std::array<float, 3> translation = _translation;

    std::tie(joint_rotvecs, translation) = clean_pose_trans(joint_rotvecs, translation);

    // update skinning parameters
    _vol_character->set_translation(translation);
    _vol_character->set_pose(joint_rotvecs);

    // perform skinning
    _vol_character->skinning(_converge);
}

// ---------------------------------------------------------------------------------------------------------------------

auto CharacterViewer::clean_pose_trans(
    std::vector<float> pose,
    std::array<float, 3> trans
) const
    -> std::tuple<std::vector<float>, std::array<float, 3> >
{
    if (_no_pelvis_rotation) {
        for (int i = 0; i < 6; ++i) {
            pose[i] = 0.0f;
        }
        // reset translation
        trans.fill(0.0f);
    }

    if (_no_extremities_rotation) {
        std::unordered_set<int> extremities = {
            // Left hand fingers
            16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
            // Right hand fingers
            37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51,
            // Toes
            55, 59
        };

        for (const auto joint_idx: extremities) {
            pose[joint_idx * 3 + 0] = 0.0f;
            pose[joint_idx * 3 + 1] = 0.0f;
            pose[joint_idx * 3 + 2] = 0.0f;
        }
    }

    return {pose, trans};
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(volanim)

// =====================================================================================================================
