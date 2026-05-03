// =====================================================================================================================
// Created by fabian on 9/19/24.
// mail@fabiankemper.de
// =====================================================================================================================

#include "lbs.h"

#include "../../volumetric_skinning/base/volumetric_boundary_fixer.h"
#include "volumetric_base/state/skinning/skinning.h"
#include "volumetric_base/state/utils.h"
#include "volumetric_skinning/neural_skinning/defgrad_skinner.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_skinning)

// =====================================================================================================================

using Layer = volumetric_base::Layer;

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

LinearBlendSkinning::LinearBlendSkinning() : SkinningBase()
{
}

// ---------------------------------------------------------------------------------------------------------------------

LinearBlendSkinning::~LinearBlendSkinning() = default;

// ---------------------------------------------------------------------------------------------------------------------

auto LinearBlendSkinning::updated_mean_layers(volumetric_base::VolumetricState* state) -> void
{
    if (state == nullptr) {
        return;
    }

    // create copies of mean meshes
    auto mean_skel = state->mean_surface_mesh(Layer::SKEL);
    auto mean_musc = state->mean_surface_mesh(Layer::MUSC);
    auto mean_skin = state->mean_surface_mesh(Layer::SKIN);
    diffgeo_solver = volumetric_base::diffgeo::DifferentialGeometrySolver({mean_skel, mean_musc, mean_skin});

    // dummy output
    _result_skel.set_mesh(mean_skel);
    _result_musc.set_mesh(mean_musc);
    _result_skin.set_mesh(mean_skin);
}

// ---------------------------------------------------------------------------------------------------------------------

auto LinearBlendSkinning::updated_input_layers(volumetric_base::VolumetricState* state) -> void
{
    if (state == nullptr) {
        return;
    }
    if (not state->is_initialized(true)) {
        return; // skip loading layer
    }

    // target volumina and skeleton
    pmp::SurfaceMesh _target_skel = state->get_layers().to_mesh(Layer::SKEL);
    pmp::SurfaceMesh  _target_musc_skel = state->get_layers().to_mesh(Layer::SKEL);
    pmp::SurfaceMesh _target_musc_musc = state->get_layers().to_mesh(Layer::MUSC);
    pmp::SurfaceMesh _target_skin_musc = state->get_layers().to_mesh(Layer::MUSC);
    pmp::SurfaceMesh _target_skin_skin = state->get_layers().to_mesh(Layer::SKIN);


    volumetric_base::LayeredMesh m = state->get_layers();
    volumetric_base::cleanMesh(m);
    volumetric_skinning::VolumetricBoundaryFixer fixer(m);

    pmp::SurfaceMesh oldSkin = state->get_layers().to_mesh(volumetric_base::Layer::SKIN);
    pmp::SurfaceMesh newSkel = state->get_layers().to_mesh(volumetric_base::Layer::SKEL);
    pmp::SurfaceMesh newMuscle = state->get_layers().to_mesh(volumetric_base::Layer::MUSC);
    pmp::SurfaceMesh newSkin = state->get_layers().to_mesh(volumetric_base::Layer::SKIN);

    prism_adapter::transfer_to_surface_mesh(newSkel, m, volumetric_base::Layer::SKEL);
    prism_adapter::transfer_to_surface_mesh(newMuscle, m, volumetric_base::Layer::MUSC);
    prism_adapter::transfer_to_surface_mesh(newSkin, m, volumetric_base::Layer::SKIN);

    fixer.reattach_body_parts(state, oldSkin, newSkin);
    fixer.transfer_fixes(state, newSkin, newMuscle, newSkel);




    // set final positions of layers
    _result_skel.positions_from_baked_mesh(newSkel);
    _result_musc.positions_from_baked_mesh(newMuscle);
    _result_skin.positions_from_baked_mesh(newSkin);
}

// ---------------------------------------------------------------------------------------------------------------------

auto LinearBlendSkinning::updated_pose(volumetric_base::VolumetricState* state) -> void
{
    if (state == nullptr) {
        return;
    }

    volumetric_base::skinning::lbs(state, _result_skel, Layer::SKEL);
    volumetric_base::skinning::lbs(state, _result_musc, Layer::MUSC);
    volumetric_base::skinning::lbs(state, _result_skin, Layer::SKIN);
}

// ---------------------------------------------------------------------------------------------------------------------

auto LinearBlendSkinning::perform_skinning(
    volumetric_base::VolumetricState* state,
    volumetric_base::SkinnableMesh& skel, volumetric_base::SkinnableMesh& musc, volumetric_base::SkinnableMesh& skin,
    [[maybe_unused]] bool converge
) -> void
{
    if (state == nullptr) {
        return;
    }

    // copy from internal skinnable mesh
    skel.copy_skinning_from_skinnable_mesh(_result_skel);
    musc.copy_skinning_from_skinnable_mesh(_result_musc);
    skin.copy_skinning_from_skinnable_mesh(_result_skin);
}

// ---------------------------------------------------------------------------------------------------------------------

auto LinearBlendSkinning::_fix_collapsed_vertices(
    const volumetric_base::VolumetricState* state) -> void
{
    if (state == nullptr) {
        return;
    }

    // apply hand fix
    auto skel_global_pos = _result_skel.mesh().vertex_property<pmp::Point>(pmp_character::properties::vertexGlobalPosition);
    auto musc_global_pos = _result_musc.mesh().vertex_property<pmp::Point>(pmp_character::properties::vertexGlobalPosition);
    auto skin_global_pos = _result_skin.mesh().vertex_property<pmp::Point>(pmp_character::properties::vertexGlobalPosition);

    for (auto v_idx: state->collapsed_vertices) {
        const auto vertex = pmp::Vertex(v_idx);
        musc_global_pos[vertex] = skel_global_pos[vertex];
        skin_global_pos[vertex] = skel_global_pos[vertex];
    }
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================
