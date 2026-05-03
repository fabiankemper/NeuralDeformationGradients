//
// Created by gerrit on 5/26/25.
//


#ifdef WITH_IMGUI
#include <imgui.h>
#endif
#include "StableNeoHookeanSkinner.h"
#include <pmp/surface_mesh.h>

#include "../base/basic_skinning.h"
#include "../base/volumetric_boundary_fixer.h"
#include "pmp_character/fbxfile/fbx_file.h"
#include "volumetric_base/state/skinning/skinning.h"
#include "volumetric_skinning/neural_skinning/defgrad_skinner.h"

/***********************************************************************************************************************
 *
 *      Implements the virtual skinning API for skinning using Stable Neo
 * Hookean Elasticity (Smith et al 2019) Theres not much math here, just API
 *
 **********************************************************************************************************************/

namespace volumetric_skinning::neo_hookean {
    void NeoHookeanSkinner::perform_skinning(
        volumetric_base::VolumetricState *state,
        volumetric_base::SkinnableMesh &skel,
        volumetric_base::SkinnableMesh &musc,
        volumetric_base::SkinnableMesh &skin,
        bool converge
    ) {

        if (converge) {
            currently_animated_mesh_copy = _animator->animate(*state);
        }

        volumetric_base::skinning::lbs(state,skel, volumetric_base::Layer::SKEL);

        pmp::SurfaceMesh new_skel = skel.bake_mesh();
        pmp::SurfaceMesh new_muscle = skel.bake_mesh();
        pmp::SurfaceMesh new_skin = skel.bake_mesh();


        prism_adapter::transfer_to_surface_mesh(new_skel, currently_animated_mesh_copy, volumetric_base::Layer::SKEL);
        prism_adapter::transfer_to_surface_mesh(new_muscle, currently_animated_mesh_copy, volumetric_base::Layer::MUSC);
        prism_adapter::transfer_to_surface_mesh(new_skin, currently_animated_mesh_copy, volumetric_base::Layer::SKIN);

        skel.skinning_from_baked_mesh(new_skel);
        musc.skinning_from_baked_mesh(new_muscle);
        skin.skinning_from_baked_mesh(new_skin);

    }

    //======================================================================================================================

    void NeoHookeanSkinner::updated_input_layers(volumetric_character::VolumetricState *state) {
        if(!_combinator)
        {
            std::cerr << "Called mesh combination without having instantiated mean meshes. "<< std::endl;
            return;
        }
        rest_pose_mesh= state->get_layers();
        volumetric_base::cleanMesh(rest_pose_mesh);
        basic_skinning::setup_mesh_properties_for_cleaned_mesh(rest_pose_mesh,state->get_skinning_parameters(volumetric_base::Layer::SKEL));
        delete _animator;
        _animator = new StableNeoHookeanSkinning(rest_pose_mesh);

        auto constrained_mapping = _animator->constrained_mapping;
    }

    //======================================================================================================================

    void NeoHookeanSkinner::updated_mean_layers(volumetric_base::VolumetricState *state) {
        const volumetric_base::LayeredMesh mean = state->mean_mesh();

        delete _combinator;
        _combinator = new volumetric_base::diffgeo::DifferentialGeometrySolver(mean);
        _combinator->initialize(state);
    }

    //======================================================================================================================

    void NeoHookeanSkinner::updated_pose(volumetric_base::VolumetricState *state) {
        if(_animator==nullptr)
        {
            std::cerr << "Called mesh combination without having instantiated mean meshes. "<< std::endl;
            exit(0);

        }
        currently_animated_mesh_copy = rest_pose_mesh;

        pmp::SurfaceMesh skeleton = state->get_layers().to_mesh(volumetric_base::Layer::SKEL);
        basic_skinning::setup_mesh_properties_for_cleaned_mesh(currently_animated_mesh_copy,  state->get_skinning_parameters(volumetric_base::Layer::SKEL));
        basic_skinning::linear_blend_skin(currently_animated_mesh_copy, state);
    }

    //======================================================================================================================

    void NeoHookeanSkinner::imgui_update(volumetric_character::VolumetricState *state) {
#ifdef WITH_IMGUI
        SkinningBase::imgui_update(state);

        if (ImGui::CollapsingHeader("Material Parameters")) {
            ImGui::SliderFloat("Skin lambda",&_animator->neo_hookean_energy.skin_lame_lambda,0,1000);
            ImGui::SliderFloat("Skin mu",&_animator->neo_hookean_energy.skin_lame_mu,0,1000 );
            ImGui::SliderFloat("Muscle lambda",&_animator->neo_hookean_energy.muscle_lame_lambda,0,1000);
            ImGui::SliderFloat("Muscle mu",&_animator->neo_hookean_energy.muscle_lame_mu,0,1000 );
            ImGui::SliderFloat("Skin Bending Energy", &_animator->skin_bending_energy_weight,1e-7,1e-2,"%.3g" ,ImGuiSliderFlags_Logarithmic);
            ImGui::SliderFloat("Skin Stretching Energy", &_animator->skin_stretching_energy_weight,1e-7,1e-2,"%.3g" ,ImGuiSliderFlags_Logarithmic);
        }
        constexpr const char* labels[] = {"Prism", "Quadrature", "Tets"};
        constexpr int count = std::size(labels);

        if (ImGui::Button("Newton Step")) {
            doStep();
            state->set_skinning_result(this->currently_animated_mesh_copy);
        }
        if (ImGui::BeginCombo("Discretization Type", labels[static_cast<int>(_animator->discretization)])) {
            for (int i = 0; i < count; i++) {
                bool selected = (static_cast<int>(_animator->discretization) == i);
                if (ImGui::Selectable(labels[i], selected))
                    _animator->discretization = static_cast<Discretization>(i);
                if (selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
#endif
}

//======================================================================================================================

void NeoHookeanSkinner::doStep() {
    if (energies.empty()) {
        double energy = _animator->compute_energy(this->currently_animated_mesh_copy);
        energies.emplace_back(energy);
    }
    std::vector<double> new_energies = _animator->do_skinning_steps(this->currently_animated_mesh_copy, 1);
    for (double energy : new_energies) {
        energies.emplace_back(energy);
    }

}
} // nonlinear_skinning
// volumetric_skinning