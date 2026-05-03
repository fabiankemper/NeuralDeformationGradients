// =====================================================================================================================
// Created by fabian on 4/30/25.
// mail@fabiankemper.de
// =====================================================================================================================


#ifdef WITH_IMGUI
#include <imgui.h>
#endif
#include "../base/basic_skinning.h"
#include "../base/volumetric_boundary_fixer.h"
#include "pmp/stop_watch.h"
#include "vertex_skinner.h"
#include "volumetric_base/diffgeo/prism_diffgeo.h"
#include "volumetric_base/state/skinning/skinning.h"
#include "volumetric_base/state/utils.h"
#include "volumetric_base/utility/imgui_util.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_skinning)
NAMESPACE_BEGIN(network)

// =====================================================================================================================
using LayeredMesh = volumetric_base::LayeredMesh;
using Layer = volumetric_base::Layer;

using SkinnableMesh = volumetric_base::SkinnableMesh;
using VolumetricState = volumetric_base::VolumetricState;
// ---------------------------------------------------------------------------------------------------------------------

VertexNetworkSkinner::VertexNetworkSkinner() : predictor(volanim::filenames::vertex_onnx_model().wstring()) {
  diffeo_solver = volumetric_base::diffgeo::DifferentialGeometrySolver();
  VertexNetworkSkinner::wake_up();
}

// ---------------------------------------------------------------------------------------------------------------------

VertexNetworkSkinner::~VertexNetworkSkinner()
= default;

// ---------------------------------------------------------------------------------------------------------------------

auto VertexNetworkSkinner::updated_mean_layers(VolumetricState* state) -> void
{
    diffeo_solver.initialize(state);
}

// ---------------------------------------------------------------------------------------------------------------------

auto VertexNetworkSkinner::updated_input_layers(VolumetricState* state) -> void
{
    const LayeredMesh& skel = state->get_layers();

    _current_a_pose_mesh = skel;
    volumetric_base::clean_mesh(_current_a_pose_mesh, state->collapsed_vertices);
    predictor.set_body(_current_a_pose_mesh.toVertexVector().cast<float>());

}

// ---------------------------------------------------------------------------------------------------------------------

auto VertexNetworkSkinner::updated_pose(VolumetricState* state) -> void
{
    (void) state;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VertexNetworkSkinner::perform_skinning(
    VolumetricState* state,
    SkinnableMesh& skel,
    SkinnableMesh& musc,
    SkinnableMesh& skin,
    [[maybe_unused]] bool converge
) -> void
{
    LayeredMesh animated_mesh;
    if (show_correctives) {
        animated_mesh = this->skin_blendshape(state,_current_a_pose_mesh);
    }
    else {
        volumetric_base::skinning::lbs(state,skel,Layer::SKEL);
        animated_mesh = this->skin(state, _current_a_pose_mesh);
    }


    pmp::SurfaceMesh new_skel;
    pmp::SurfaceMesh new_muscle;
    pmp::SurfaceMesh new_skin;

    // A bit annoying, this is just to add hands etc. correctly
    if (show_correctives) {
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

    prism_adapter::transfer_to_surface_mesh(new_skel, animated_mesh, Layer::SKEL);
    prism_adapter::transfer_to_surface_mesh(new_muscle, animated_mesh, Layer::MUSC);
    prism_adapter::transfer_to_surface_mesh(new_skin, animated_mesh, Layer::SKIN);

    skel.skinning_from_baked_mesh(new_skel);
    musc.skinning_from_baked_mesh(new_muscle);
    skin.skinning_from_baked_mesh(new_skin);
}

// ---------------------------------------------------------------------------------------------------------------------

auto VertexNetworkSkinner::perform_blendshape_skinning(
    VolumetricState* state,
    SkinnableMesh& skel,
    SkinnableMesh& musc,
    SkinnableMesh& skin
) -> void
{
    LayeredMesh animated_mesh = this->skin_blendshape(state, _current_a_pose_mesh);

    VolumetricBoundaryFixer fixer(animated_mesh);

    pmp::SurfaceMesh oldSkin = state->get_layers().to_mesh(Layer::SKIN);
    pmp::SurfaceMesh newSkel = state->get_layers().to_mesh(Layer::SKEL);
    pmp::SurfaceMesh newMuscle = state->get_layers().to_mesh(Layer::MUSC);
    pmp::SurfaceMesh newSkin = state->get_layers().to_mesh(Layer::SKIN);

    prism_adapter::transfer_to_surface_mesh(newSkel, animated_mesh, Layer::SKEL);
    prism_adapter::transfer_to_surface_mesh(newMuscle, animated_mesh, Layer::MUSC);
    prism_adapter::transfer_to_surface_mesh(newSkin, animated_mesh, Layer::SKIN);

    fixer.reattach_body_parts(state, oldSkin, newSkin);
    fixer.transfer_fixes(state, newSkin, newMuscle, newSkel);

    skel.skinning_from_baked_mesh(newSkel);
    musc.skinning_from_baked_mesh(newMuscle);
    skin.skinning_from_baked_mesh(newSkin);
}

// ---------------------------------------------------------------------------------------------------------------------

auto VertexNetworkSkinner::wake_up() -> void
{

}

// ---------------------------------------------------------------------------------------------------------------------

auto VertexNetworkSkinner::load_model(const std::filesystem::path& model_filename) -> void
{
    _model_path = model_filename;
    wake_up();
}

// ---------------------------------------------------------------------------------------------------------------------

auto VertexNetworkSkinner::imgui_update([[maybe_unused]]  VolumetricState* state) -> void
{
#ifdef WITH_IMGUI
    const std::string show = "Currently using: " + _model_path.filename().string();

    if (ImGui::Checkbox("Show Pose Correctives", &show_correctives)) {
        dirty = true;
    }
    imgui_util::HelpMarker(
        "If toggled, skinning is disabled and only the correctives predicted by "
        "the neural network are shown.");


    if (ImGui::Checkbox("Disable Network Predictions", &disable_correctives)) {
        dirty = true;
    }
    imgui_util::HelpMarker(
        "If toggled, the network is disabled and only the baseline results (LBS) are shown.");

    ImGui::Text("%s", show.c_str());

#endif
}
auto VertexNetworkSkinner::predict(const Eigen::MatrixXd &rotations) -> void {
        predictor.predict(matrix_util::flatten(rotations).cast<float>());
    if (disable_correctives)
        predictor.final_correctives.setZero();

}

// ---------------------------------------------------------------------------------------------------------------------

auto VertexNetworkSkinner::skin(const VolumetricState *state,
    const LayeredMesh& mesh
) -> LayeredMesh
{
    pmp::StopWatch watch; watch.start();
    LayeredMesh m = mesh;
    basic_skinning::setup_mesh_properties_for_cleaned_mesh(m, state->get_skinning_parameters(Layer::SKEL));
    const Eigen::VectorXd position_vec = m.toVertexVector();
    const Eigen::MatrixXd rotations = volumetric_base::utils::local_rotation_matrix(state);
    try {

        predict(rotations);
        const Eigen::MatrixXd vertex_mat = mesh.toVertexMatrix();
        m.set_mesh_from_matrix(predictor.final_correctives + vertex_mat);
        basic_skinning::setup_mesh_properties_for_cleaned_mesh(m, state->get_skinning_parameters(Layer::SKEL));
        basic_skinning::linear_blend_skin(m, state);
        watch.stop();
    } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
    watch.stop();
    return m;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VertexNetworkSkinner::skin_blendshape(
    const VolumetricState* state,
    const LayeredMesh& mesh_to_skin
)
    -> LayeredMesh
{
    LayeredMesh m = mesh_to_skin;

    const Eigen::MatrixXd rotations = volumetric_base::utils::local_rotation_matrix(state);

    predict(rotations);
    m.set_mesh_from_matrix(predictor.final_correctives + mesh_to_skin.toVertexMatrix());
    return m;
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(network)
NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================
