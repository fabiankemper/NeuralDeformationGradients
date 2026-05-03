// =====================================================================================================================
//
// Created by Fabian Kemper on 2025-04-10.
//
// =====================================================================================================================

#include "utils.h"

#include <npy.hpp>
#include <pmp_character/fbxfile/fbx_file.h>

#include "volumetric_base/rbf_warp/rbf_warp_stephan.h"

#include "io.h"
#include "filenames.h"
#include "pmp_ext/pmp_matrix_ext.h"
#include "retarget_skeleton/mean_value_coordinates.h"
#include "retarget_skeleton/retarget_skeleton.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(utils)

// =====================================================================================================================


// ---------------------------------------------------------------------------------------------------------------------

auto local_rotation_matrix(const VolumetricState* state) -> Eigen::MatrixXd
{
    if (state == nullptr) {
        throw std::runtime_error("state in local_rotation_matrix is null");
    }
    auto& rig = state->rig;
    Eigen::MatrixXd rotations(rig.joints_.size() * 3,3);

    OMP_PARALLEL_FOR()
    for (int idx = 0; idx < static_cast<int>(rig.joints_.size()); idx++) {
        Eigen::Matrix4d joint_transform = rig.joints_[idx]->local_;
        rotations.block<3, 3>(idx*3,0) = joint_transform.block<3, 3>(0,0);
    }
    return rotations;
}

auto local_rotation_matrix(const VolumetricState *state, Eigen::MatrixXd &result) -> void {
    if (state == nullptr) {
        throw std::runtime_error("state in local_rotation_matrix is null");
    }
    auto& rig = state->rig;

    if (result.rows() == 0)
        result.resize(rig.joints_.size() * 3,3);

    OMP_PARALLEL_FOR()
    for (long idx = 0; idx < static_cast<long>(rig.joints_.size()); idx++) {
        Eigen::Matrix4d joint_transform = rig.joints_[idx]->local_;
        result.block<3, 3>(idx*3,0) = joint_transform.block<3, 3>(0,0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto pose_from_rotation_matrices(VolumetricState* state, const Eigen::MatrixXd& rotations) -> void
{
    if (state == nullptr) {
        throw std::runtime_error("state in pose_from_rotation_matrices is null");
    }

    auto& rig = state->rig;

    rig.reset();
    //OMP_PARALLEL_FOR()
    for (unsigned int idx = 0; idx < rig.joints_.size(); idx++) {
        auto& joint_transform = rig.joints_[idx]->local_;
        // joint_transform.block<3, 3>(0,0) = rotations.block<3, 3>(idx * 3,0);
        for (unsigned int i = 0; i < 3; i++) {
            for (unsigned int j = 0; j < 3; j++) {
                joint_transform(i, j) = static_cast<float>(rotations(idx * 3 + i, j));
            }
        }
        rig.joints_[idx]->local_ = rig.joints_[idx]->bind_pose_local * joint_transform;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * This is the ONLY method you should ever use.
 * The following methods are helper methods for setting up the volumetric state and can mess up a lot of things
 *
 * @param state Current VolumetricState for setup
 * @param data_dir Data Directory (data of this main repository)
 * @param gender Male or Female for reading some skeleton specifics
 */
auto StateModification::setup_state(VolumetricState* state, const std::filesystem::path& data_dir, const Gender gender) -> void
{
    if (state == nullptr) {
        throw std::invalid_argument("state is null pointer.");
    }

    if (not exists(data_dir)) {
        throw std::invalid_argument("data_dir does not exist.");
    }

    // read settings
    auto& settings = volanim::config::settings;

    // set gender in state
    state->gender = gender;

    // first read collapsed vertices
    read_collapsed_vertices(state);

    // paths for rig and skinning weights
    std::filesystem::path template_skin_path = volanim::filenames::template_skin_m();
    std::filesystem::path template_skel_wrap_path = volanim::filenames::template_skel_wrap_m();

    if (gender == Gender::FEMALE) {
        template_skin_path = volanim::filenames::template_skin_f();
        template_skel_wrap_path = volanim::filenames::template_skel_wrap_f();
    }

    // read basic character for skinning parameters and rig
    const auto skin_character = std::make_shared<pmp_character::Character>();
    auto fbx_file = pmp_character::FBX_file();
    fbx_file.read(skin_character.get(), template_skin_path);

    // read skeleton wrap to set skinning weights for skeleton wrap
    const auto skel_wrap_character = std::make_shared<pmp_character::Character>();
    fbx_file.read(skel_wrap_character.get(), template_skel_wrap_path);

    // set parameters from character
    // state->rig = Skeleton(skin_character->skeleton_); // copy skeleton from skin
    state->rig = Skeleton(skel_wrap_character->skeleton_);

    // alternative: load wrap weights from npy
    // read_rig(state, data_dir, gender); // todo: this can be used, if we do not load the fbx file

    // set_skinning_parameters_from_character(state, character.get());

    // todo: setup mvc weights
    // todo: how (in which layer) to store those weights for later use?

    // template
    read_highres_meshes(state, gender);
    read_highres_wraps(state, gender);

    // template layers
    read_mean_layers(state, gender); // legacy?
    read_template_layers(state, gender);

    // setup skinning weights
    set_skinning_parameters_from_character_for_layer(state, skel_wrap_character.get(), Layer::SKEL, volanim::filenames::template_node_skeleton_wrap.data());
    set_skinning_parameters_from_character_for_layer(state, skin_character.get(), Layer::MUSC, volanim::filenames::template_node_skin.data());
    set_skinning_parameters_from_character_for_layer(state, skin_character.get(), Layer::SKIN, volanim::filenames::template_node_skin.data());

    // todo: skinning weights might be read from skinning weights dictionary!!

    // set skinning weights to template layers (these are used for creating the skinnable meshes!)
    set_skinning_weights_to_template_layers(state);

    // setup skeleton retargeting
    RetargetSkeleton::setup_retargeting(state);

    // set bindpose (if not disabled)
    // if (not state->options().contains("with_bindpose") || state->options()["with_bindpose"].get<bool>()) {
    if (settings.load_bindpose) {
        read_bindpose_from_file(state, volanim::filenames::template_bindpose());
    }

    // set all layers to mean
    state->set_layers_from_mean();

    // retarget to mean layers
    RetargetSkeleton::retarget_skeleton(state);

    // update revision (for safety?)
    state->update_revision();

    // todo: check consistency
}

// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::retarget_skeleton(VolumetricState* state) -> void
{
    if (state == nullptr) {
        throw std::invalid_argument("state is null pointer.");
    }

    // perform retargeting
    RetargetSkeleton::retarget_skeleton(state);
    calculate_new_highres_skeleton(state);
}

// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::set_pose(VolumetricState* state, const std::vector<float>& pose_rotvec) -> void
{
    if (state == nullptr) {
        throw std::invalid_argument("set_pose: state is null pointer.");
    }

    state->set_pose(pose_rotvec);

    // actually perform animation on skeleton / rig
    const auto& rig = state->rig;

    // check dimensions
    if (pose_rotvec.size() != rig.joints_.size() * 3) {
        std::cerr << "Joint angles sizes mismatch. Got " <<  pose_rotvec.size() << " expected " << rig.joints_.size() * 3 << "\n";
        return;
    }

    // set local rotation matrices
    for (size_t index = 0; index < rig.joints_.size(); ++index) {
        auto* joint = rig.joints_[index];

        // convert to rotation matrix
        const pmp::vec3 rodrigues_angles (pose_rotvec[index * 3 + 0], pose_rotvec[index * 3 + 1], pose_rotvec[index * 3 + 2]);
        // convert to radian
        // rodrigues_angles *= M_PI / 180.0f;
        pmp::Mat4<float> rotation_mat = pmp_ext::rodrigues_to_mat4(rodrigues_angles);

        if (joint->name_ == "root") {
            joint->bind_pose_local[12] = 0.0;
            joint->bind_pose_local[13] = 0.0;
            joint->bind_pose_local[14] = 0.0;

            // keep translation
            rotation_mat[12] = joint->local_[12];
            rotation_mat[13] = joint->local_[13];
            rotation_mat[14] = joint->local_[14];
        }

        joint->set_rotation(rotation_mat, /* perform_kinematics=*/false);
    }

    // perform kinematics now
    state->rig.forward_kinematics();
}

// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::set_translation(VolumetricState* state, const pmp::vec3& translation) -> void
{
    if (state == nullptr) {
        throw std::invalid_argument("set_translation: state is null pointer.");
    }
    if (translation.size() != 3) {
        throw std::invalid_argument("set_translation: translation size mismatch. Got " + std::to_string(translation.size()) + " expected " + std::to_string(3));
    }

    // actually perform animation on skeleton / rig
    const auto& rig = state->rig;

    rig.root_->set_local_translation(translation);

    // perform kinematics now
    state->rig.forward_kinematics();
}


// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::calculate_new_highres_skeleton(VolumetricState* state) -> void
{
    if (state == nullptr) { return; }

    // target skeleton wrap
    const auto target_skel_wrap = state->get_layers().to_mesh(Layer::SKEL);

    if (state->highres_skeleton_warp == nullptr) {
        state->highres_skeleton_warp = new warping::RBFWarpStephan();

        // high-res template skeleton
        const auto& template_wrap = state->get_highres_wrap(Layer::SKEL);
        state->highres_skeleton_warp->init_warp(template_wrap, volanim::filenames::rbf_centers_hires_skeleton);
    }

    // create a new high-res skeleton for skinning
    const auto warped_highres_skeleton = state->highres_skeleton_warp->apply_warp(target_skel_wrap, state->get_highres_mesh(Layer::SKEL));
    const auto skinnable_highres_skeleton = SkinnableMesh(warped_highres_skeleton);
    state->set_scaled_highres_skeleton(skinnable_highres_skeleton);
}

// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::set_skinning_parameters_from_character_for_layer(VolumetricState* state, Character* character, const Layer layer, const std::string& mesh_name) -> void
{
    // get the biggest mesh from character
    auto& skinning_mesh = character->get_mesh_by_name(mesh_name).mesh;

    auto skin_vdepends = skinning_mesh.vertex_property<ivec8>(pmp_character::properties::vertexSkinningDepends);
    auto skin_vweights = skinning_mesh.vertex_property<vec8>(pmp_character::properties::vertexSkinningWeights);

    const auto n_vertices = skinning_mesh.n_vertices();

    std::array<SkinningParameters, 3> skinning_parameters;

    // resize buffer
    skinning_parameters[static_cast<std::underlying_type_t<Layer>>(layer)].resize(n_vertices);

    // copy parameters
    for (auto vertex : skinning_mesh.vertices()) {
        skinning_parameters[static_cast<std::underlying_type_t<Layer>>(layer)][vertex.idx()] = std::make_tuple(skin_vdepends[vertex], skin_vweights[vertex]);
    }

    // set in state
    state->set_skinning_parameters(layer, skinning_parameters[static_cast<std::underlying_type_t<Layer>>(layer)]);

    state->update_revision();
}

// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::set_rig_from_character(VolumetricState* state, const Character* character) -> void
{
    if (state == nullptr) { return; }
    auto& rig = state->rig;

    // clear current rig
    for (const auto* joint : rig.joints_) {
        delete joint;
    }
    rig.joints_.clear();
    rig.root_ = nullptr;
    rig.selected_joint_ = -1;

    // deep-copy animation rig
    rig = character->skeleton_;
}

// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::set_skinning_weights_to_mesh(const VolumetricState* state, pmp::SurfaceMesh& mesh, const Layer layer) -> void
{
    if (state == nullptr) { return; }

    const auto& skinning_weights = state->get_skinning_parameters(layer);
    if (skinning_weights.size() != mesh.n_vertices()) {
        std::cerr << "skinning_weights.size() = " << skinning_weights.size() << " != mesh.n_vertices() = " << mesh.n_vertices() << std::endl;
        return;
    }

    // add properties
    auto v_depends = mesh.vertex_property<ivec8>(pmp_character::properties::vertexSkinningDepends);
    auto v_weights = mesh.vertex_property<vec8>(pmp_character::properties::vertexSkinningWeights);
    for (auto vertex : mesh.vertices()) {
        const auto& [depend, weight] = skinning_weights[vertex.idx()];
        v_depends[vertex] = depend;
        v_weights[vertex] = weight;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::set_skinning_weights_to_template_layers(VolumetricState* state) -> void
{
    if (state == nullptr) {
        throw std::invalid_argument("state is null pointer.");
    }

    auto& skel = state->get_template_layer_unsafe(Layer::SKEL);
    auto& musc = state->get_template_layer_unsafe(Layer::MUSC);
    auto& skin = state->get_template_layer_unsafe(Layer::SKIN);

    set_skinning_weights_to_mesh(state, skel, Layer::SKEL);
    set_skinning_weights_to_mesh(state, musc, Layer::MUSC);
    set_skinning_weights_to_mesh(state, skin, Layer::SKIN);
}

// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::set_bindpose(VolumetricState* state, const std::vector<float>& bindpose_rotvec) -> void
{
    if (state == nullptr) {
        throw std::invalid_argument("state is null pointer.");
    }

    auto& rig = state->rig;

    if (bindpose_rotvec.size() != rig.joints_.size() * 3) {
        std::cerr << "Bindpose angles do not match skeleton joint count.\n";
        return;
    }

    // copy joint angles
    state->set_bindpose(bindpose_rotvec);

    // set bindpose
    set_pose(state, bindpose_rotvec);
    rig.set_current_pose_as_bindpose(/*perform_kinematics=*/true);

    // perform skinning
    rig.forward_kinematics();
}

// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::clear_bindpose(VolumetricState* state) -> void
{
    if (state == nullptr) {
        throw std::invalid_argument("state is null pointer.");
    }

    auto& rig = state->rig;
    rig.clear_bindpose();

    // reset angles in state
    auto current_bindpose = state->get_bindpose();
    std::ranges::fill(current_bindpose, 0.0f);
    state->set_bindpose(current_bindpose);
}

// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::read_highres_meshes(VolumetricState* state, const Gender gender) -> void
{
    if (state == nullptr) {
        throw std::runtime_error("read_highres_meshes: state is null");
    }
    if (gender == Gender::MALE) {
        io::state_read_highres_mesh(state, Layer::SKEL, volanim::filenames::template_skel_hires_m(), volanim::filenames::template_node_skeleton_hires.data());
        io::state_read_highres_mesh(state, Layer::MUSC, volanim::filenames::template_musc_hires_m(), volanim::filenames::template_node_musc_hires.data());
        io::state_read_highres_mesh(state, Layer::SKIN, volanim::filenames::template_skin_m(), volanim::filenames::template_node_skin.data());
    } else {
        io::state_read_highres_mesh(state, Layer::SKEL, volanim::filenames::template_skel_hires_f(), volanim::filenames::template_node_skeleton_hires.data());
        io::state_read_highres_mesh(state, Layer::MUSC, volanim::filenames::template_musc_hires_f(), volanim::filenames::template_node_musc_hires.data());
        io::state_read_highres_mesh(state, Layer::SKIN, volanim::filenames::template_skin_f(), volanim::filenames::template_node_skin.data());
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::read_highres_wraps(VolumetricState* state, const Gender gender) -> void
{
    if (state == nullptr) {
        throw std::runtime_error("read_highres_wraps: state is null");
    }

    auto read_mesh = [](const Path& filename) -> pmp::SurfaceMesh {
        pmp::SurfaceMesh mesh;
        pmp::read(mesh, filename);
        return mesh;
    };

    pmp::SurfaceMesh skel_wrap, musc_wrap, skin_wrap;

    // load wraps, these are ALWAYS male, because we don't have a female high-res version
    if (gender == Gender::MALE) {
        skel_wrap = read_mesh(volanim::filenames::template_layer_skel_m());
        musc_wrap = read_mesh(volanim::filenames::template_layer_musc_m());
        skin_wrap = read_mesh(volanim::filenames::template_layer_skin_m());
    } else {
        skel_wrap = read_mesh(volanim::filenames::template_layer_skel_f());
        musc_wrap = read_mesh(volanim::filenames::template_layer_musc_f());
        skin_wrap = read_mesh(volanim::filenames::template_layer_skin_f());
    }

    // set wraps
    state->set_highres_wrap(Layer::SKEL, skel_wrap);
    state->set_highres_wrap(Layer::MUSC, musc_wrap);
    state->set_highres_wrap(Layer::SKIN, skin_wrap);
}

// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::read_mean_layers(VolumetricState* state, const Gender gender) -> void
{
    if (state == nullptr) {
        throw std::runtime_error("read_mean_layers: state is null");
    }

    const auto skeleton_mean_fn = volanim::filenames::mean_layer(Layer::SKEL, gender);
    const auto muscle_mean_fn = volanim::filenames::mean_layer(Layer::MUSC, gender);
    const auto skin_mean_fn = volanim::filenames::mean_layer(Layer::SKIN, gender);

    if (not exists(skeleton_mean_fn) or not exists(muscle_mean_fn) or not exists(skin_mean_fn)) {
        std::cerr << "Could not find layer files.\n";
        return;
    }

    SurfaceMesh mean_skel, mean_musc, mean_skin;

    read(mean_skel, skeleton_mean_fn);
    read(mean_musc, muscle_mean_fn);
    read(mean_skin, skin_mean_fn);

    // generate new character renderer
    state->set_mean_meshes(mean_skel, mean_musc, mean_skin);
}

// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::read_template_layers(VolumetricState* state, const Gender gender) -> void
{
    if (state == nullptr) {
        throw std::runtime_error("read_template_layers: state is null");
    }

    const auto skeleton_wrap_fn = volanim::filenames::template_layer(Layer::SKEL, gender);
    const auto muscle_wrap_fn = volanim::filenames::template_layer(Layer::MUSC, gender);
    const auto skin_template_fn = volanim::filenames::template_layer(Layer::SKIN, gender);

    if (not exists(skeleton_wrap_fn) or not exists(muscle_wrap_fn) or not exists(skin_template_fn)) {
        std::cerr << "Could not find layer files.\n";
        return;
    }

    SurfaceMesh skel, musc, skin;

    read(skel, skeleton_wrap_fn);
    read(musc, muscle_wrap_fn);
    read(skin, skin_template_fn);

    state->set_template_layers(skel, musc, skin);
}

// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::read_rig(VolumetricState* state, const Gender gender) -> void
{
    if (state == nullptr) {
        throw std::runtime_error("read_rig: state is null");
    }

    io::read_rig_from_file(state, volanim::filenames::template_rig(gender));
}

// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::read_collapsed_vertices(VolumetricState* state) -> void
{
    if (state == nullptr) {
        throw std::runtime_error("read_collapsed_vertices: state is null");
    }

    const Selection collapsed_vertices = volanim::utils::read_selection(volanim::filenames::collapsed_vertices());
    state->collapsed_vertices = collapsed_vertices;
}

// ---------------------------------------------------------------------------------------------------------------------

auto StateModification::read_bindpose_from_file(
    VolumetricState* state,
    const std::filesystem::path& bindpose_file
) -> void
{
    if (state == nullptr) {
        throw std::runtime_error("read_bindpose_from_file: state is null");
    }

    if (not exists(bindpose_file)) {
        std::cerr << "Could not read bindpose file " << bindpose_file << "\n";
        return;
    }

    // read pose angles
    const npy::npy_data npy_bindpose_angles = npy::read_npy<float>(bindpose_file.string());

    // check if joint count match
    if (npy_bindpose_angles.shape[0] != state->rig.joints_.size() * 3) {
        std::cerr << "Bindpose dimension (" << npy_bindpose_angles.shape[0] << ") does not match template joint count ("
                  << state->rig.joints_.size() * 3 << ").\n";
        return;
    }

    std::vector<float> joint_angles;
    joint_angles.resize(npy_bindpose_angles.shape[0]);

    // convert joint angles from radian to degree
    for (size_t index = 0; index < joint_angles.size(); ++index) {
        joint_angles[index] = npy_bindpose_angles.data[index];
    }

    // set bindpose
    set_bindpose(state, joint_angles);
    state->rig.forward_kinematics();
}

// ---------------------------------------------------------------------------------------------------------------------


// =====================================================================================================================

NAMESPACE_END(utils)
NAMESPACE_END(volumetric_base)

// =====================================================================================================================
