#include "defgrad_skinner.h"
#include <future>
#ifdef WITH_IMGUI
#include "imgui.h"
#endif

#include "volanim_utils/npy_utils.h"

#include "volumetric_base/state/filenames.h"
#include "volumetric_base/state/skinning/skinning.h"
#include "volumetric_base/state/utils.h"
#include "volumetric_base/utility/skinning_normalization.h"

#include "../base/basic_skinning.h"
#include "../base/volumetric_boundary_fixer.h"
#include "volumetric_base/utility/imgui_util.h"
#include "volumetric_skinning/stable_neo_hookean_elasticity/StableNeoHookeanSimulation.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_skinning)
NAMESPACE_BEGIN(network)

// =====================================================================================================================

using LayeredMesh = volumetric_base::LayeredMesh;

using SkinnableMesh = volumetric_base::SkinnableMesh;
using VolumetricState = volumetric_base::VolumetricState;

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

DefgradNetworkSkinner::DefgradNetworkSkinner() : predictor(
        std::filesystem::absolute(volanim::filenames::defgrad_onnx_model())
    )
{
    diffgeo_solver_ = new volumetric_base::diffgeo::DifferentialGeometrySolver();
    DefgradNetworkSkinner::wake_up();
}

// ---------------------------------------------------------------------------------------------------------------------

DefgradNetworkSkinner::~DefgradNetworkSkinner()
{
    delete diffgeo_solver_;
}

// ---------------------------------------------------------------------------------------------------------------------

auto DefgradNetworkSkinner::updated_mean_layers(VolumetricState* state)
    -> void
{
    delete diffgeo_solver_;

    diffgeo_solver_ = new volumetric_base::diffgeo::DifferentialGeometrySolver();
    diffgeo_solver_->initialize(state->mean_mesh());
    // this->mesh = mesh;
}

// ---------------------------------------------------------------------------------------------------------------------

auto DefgradNetworkSkinner::updated_input_layers(VolumetricState* state)
    -> void
{
    const LayeredMesh& skel = state->get_layers();

    mesh_ = skel;

    clean_mesh(mesh_, state->collapsed_vertices);
    predictor.set_body(mesh_.toVertexVector().cast<float>());
    delete diffgeo_solver_;
    diffgeo_solver_ = new volumetric_base::diffgeo::DifferentialGeometrySolver();
    diffgeo_solver_->initialize(mesh_);
}

// ---------------------------------------------------------------------------------------------------------------------

auto DefgradNetworkSkinner::updated_pose(VolumetricState* state) -> void
{
    // omit warning
    (void) state;
}

// ---------------------------------------------------------------------------------------------------------------------

auto DefgradNetworkSkinner::update_prediction(
    const Eigen::MatrixXd& rotations) -> void
{
    if (!disable_network_predictions_) {
        predictor.predict(matrix_util::flatten(rotations).cast<float>());
    } else {
        for (int i = 0; i < predictor.final_defgrads.rows() / 3; i++) {
            predictor.final_defgrads.block<3, 3>(i * 3, 0) = Eigen::Matrix3d::Identity();
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto DefgradNetworkSkinner::perform_skinning(
    VolumetricState* state, SkinnableMesh& skel, SkinnableMesh& musc,
    SkinnableMesh& skin, [[maybe_unused]] bool converge) -> void
{
    if (show_pose_correctives_) {
        animated_mesh_ = this->skin_pose_corrective(state);
    } else {
        volumetric_base::skinning::lbs(state, skel, volumetric_base::Layer::SKEL);
        animated_mesh_ = this->skin(state, mesh_);
    }
    pmp::SurfaceMesh new_skel;
    pmp::SurfaceMesh new_muscle;
    pmp::SurfaceMesh new_skin;

    // A bit annoying, this is just to add hands etc. correctly
    if (show_pose_correctives_) {
        new_skel = state->get_layers()
                .to_mesh(volumetric_base::Layer::SKEL);
        new_muscle = state->get_layers()
                .to_mesh(volumetric_base::Layer::SKEL);
        new_skin = state->get_layers()
                .to_mesh(volumetric_base::Layer::SKEL);
    } else {
        new_skel = skel.bake_mesh();
        new_muscle = skel.bake_mesh();
        new_skin = skel.bake_mesh();
    }

    prism_adapter::transfer_to_surface_mesh(new_skel, animated_mesh_,
                                            volumetric_base::Layer::SKEL);
    prism_adapter::transfer_to_surface_mesh(new_muscle, animated_mesh_,
                                            volumetric_base::Layer::MUSC);
    prism_adapter::transfer_to_surface_mesh(new_skin, animated_mesh_,
                                            volumetric_base::Layer::SKIN);

    skel.skinning_from_baked_mesh(new_skel);
    musc.skinning_from_baked_mesh(new_muscle);
    skin.skinning_from_baked_mesh(new_skin);
}

// ---------------------------------------------------------------------------------------------------------------------

void DefgradNetworkSkinner::wake_up()
{
    try {
        const std::filesystem::path model_path2 = std::filesystem::absolute(
            volanim::filenames::get_model_path("Defgrad.zip"));
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto DefgradNetworkSkinner::load_model(
    const std::filesystem::path& model_filename) -> void
{
    model_path = model_filename;
    wake_up();
}

// ---------------------------------------------------------------------------------------------------------------------

auto DefgradNetworkSkinner::imgui_update(
    [[maybe_unused]] volumetric_base::VolumetricState* state) -> void
{
#ifdef WITH_IMGUI

    if (ImGui::CollapsingHeader("Change PCA Parameters")) {
        for (int i = 0; i < predictor.cached_pca_result.rows(); i++) {
            if (ImGui::SliderFloat(("PCA Param " + std::to_string(i)).c_str(), &predictor.cached_pca_result[i], -5, 5))
                dirty = true;
        }
    }
    imgui_util::HelpMarker(
        "Change the PCA Parameters that are fed to the network. Note: changes only network predictions and not the body.");

    if (ImGui::Checkbox("Show Pose Correctives", &show_pose_correctives_)) {
        dirty = true;
    }
    imgui_util::HelpMarker(
        "If toggled, skinning is disabled and only the correctives predicted by "
        "the neural network are shown.");

    if (ImGui::Checkbox("Disable Network Predictions",
                        &disable_network_predictions_)) {
        dirty = true;
    }
    imgui_util::HelpMarker("Disables Network Predictions. Corresponds to "
        "Gradient Skinning in the paper.");

#endif // WITH_IMGUI
}

// ---------------------------------------------------------------------------------------------------------------------

auto DefgradNetworkSkinner::skin(const volumetric_base::VolumetricState* state,
                                 const LayeredMesh& mesh) -> LayeredMesh
{
    LayeredMesh m = mesh;
    basic_skinning::setup_mesh_properties_for_cleaned_mesh(
        m, state->get_skinning_parameters(volumetric_base::Layer::SKEL));
    volumetric_base::utils::local_rotation_matrix(state, rotations_);
    try {
        pmp::StopWatch watch_lbs, watch_http;
        basic_skinning::linear_blend_skin(m, state);
        update_prediction(rotations_);
        volumetric_base::normalization::unnormalize_defgrads(*state, m, predictor.final_defgrads);

        diffgeo_solver_->solveFromDefgradsDirect(predictor.final_defgrads, m);
        return m;
    } catch (const std::exception& e) {
        // std::async will forward exceptions, which .get() will re-throw.
        // This catch block will now catch errors from the parallel tasks as well.
        std::cerr << e.what() << std::endl;
    }

    // fallback
    std::cerr << "INFERENCE ERROR OCCURRED." << std::endl;
    return m;
}

auto DefgradNetworkSkinner::skin_no_rot(VolumetricState* state,
                                        const LayeredMesh& mesh)
    -> LayeredMesh
{
    LayeredMesh m = mesh;

    pmp::StopWatch watch;
    watch.start();
    volumetric_skinning::basic_skinning::setup_mesh_properties_for_cleaned_mesh(
        m, state->get_skinning_parameters(volumetric_base::Layer::SKEL));
    const auto rotations = volumetric_base::utils::local_rotation_matrix(state);
    try {
        std::cout << "Norot" << std::endl;
        update_prediction(rotations);
        basic_skinning::linear_blend_skin(m, state);
        LayeredMesh reconstruction =
                diffgeo_solver_->solveFromDeformationDefgrads(predictor.final_defgrads.cast<double>(), m);
        return reconstruction;
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }

    // fallback
    std::cerr << "INFERENCE ERROR OCCURRED." << std::endl;

    return m;
}

auto DefgradNetworkSkinner::predict_deformation_gradients(
    VolumetricState* state) -> Eigen::MatrixXd
{
    LayeredMesh m = mesh_;
    pmp::StopWatch miniwatch;
    miniwatch.start();
    volumetric_skinning::basic_skinning::setup_mesh_properties_for_cleaned_mesh(
        m, state->get_skinning_parameters(volumetric_base::Layer::SKEL));
    const auto rotations = volumetric_base::utils::local_rotation_matrix(state);
    const Eigen::VectorXd position_vec = m.toVertexVector();

    try {
        update_prediction(rotations);
        return predictor.final_defgrads.cast<double>();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }

    // fallback
    std::cerr << "INFERENCE ERROR OCCURRED." << std::endl;
    exit(5);
}

// ---------------------------------------------------------------------------------------------------------------------

auto DefgradNetworkSkinner::skin_pose_corrective(const VolumetricState* state)
    -> LayeredMesh
{
    LayeredMesh m = mesh_;

    volumetric_skinning::basic_skinning::setup_mesh_properties_for_cleaned_mesh(
        m, state->get_skinning_parameters(volumetric_base::Layer::SKEL));

    volumetric_base::utils::local_rotation_matrix(state, rotations_);

    try {
        update_prediction(rotations_);
        diffgeo_solver_->solveFromDefgradsDirect(predictor.final_defgrads, m);
        return m;
    } catch (const std::exception& e) {
        std::cerr << "Corrective Reconstruction Error: " << e.what() << std::endl;
        return mesh_;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(network)

NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================
