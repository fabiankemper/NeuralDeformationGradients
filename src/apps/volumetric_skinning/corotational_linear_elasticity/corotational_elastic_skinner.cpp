//
// Created by gerrit-nolte on 9/20/24.
//

#include "corotational_elastic_skinner.h"

#include "../base/basic_skinning.h"
#include "../base/volumetric_boundary_fixer.h"
#include "volumetric_base/state/skinning/skinning.h"
#include "volumetric_base/utility/skinning_normalization.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_skinning)

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

void CorotationalLinearElasticSkinner::perform_skinning(
    volumetric_character::VolumetricState* state,
    volumetric_base::SkinnableMesh& skel,
    volumetric_base::SkinnableMesh& musc,
    volumetric_base::SkinnableMesh& skin,
    [[maybe_unused]] bool converge
) {
    // VolumetricFixer operation
        if (_animator) {
            _animator->animate(_animated_mesh, *state);
        }
        volumetric_base::skinning::lbs(state,skel, volumetric_base::Layer::SKEL);

        pmp::SurfaceMesh new_skel = skel.bake_mesh();
        pmp::SurfaceMesh new_muscle = skel.bake_mesh();
        pmp::SurfaceMesh new_skin = skel.bake_mesh();

        prism_adapter::transfer_to_surface_mesh(new_skel, _animated_mesh, volumetric_base::Layer::SKEL);
        prism_adapter::transfer_to_surface_mesh(new_muscle, _animated_mesh, volumetric_base::Layer::MUSC);
        prism_adapter::transfer_to_surface_mesh(new_skin, _animated_mesh, volumetric_base::Layer::SKIN);

        skel.skinning_from_baked_mesh(new_skel);
        musc.skinning_from_baked_mesh(new_muscle);
        skin.skinning_from_baked_mesh(new_skin);
}   

void CorotationalLinearElasticSkinner::perform_blendshape_skinning(volumetric_character::VolumetricState *state, volumetric_base::SkinnableMesh &skel,
                                                 volumetric_base::SkinnableMesh &musc, volumetric_base::SkinnableMesh &skin) {
    VolumetricBoundaryFixer fixer(_animated_mesh);

    Eigen::MatrixXd defgrads = a_pose_diffgeo_solver.gradient * _animated_mesh.toVertexMatrix();
    volumetric_base::normalization::normalize_defgrads(*state,_animated_mesh, defgrads);
    volumetric_base::LayeredMesh blendshape = a_pose_diffgeo_solver.solveFromDeformationDefgrads(defgrads, _combined_a_pose_mesh);
    pmp::SurfaceMesh oldSkin = state->get_layers().to_mesh(volumetric_base::Layer::SKIN);
    pmp::SurfaceMesh newSkel = state->get_layers().to_mesh(volumetric_base::Layer::SKEL);
    pmp::SurfaceMesh newMuscle = state->get_layers().to_mesh(volumetric_base::Layer::MUSC);
    pmp::SurfaceMesh newSkin = state->get_layers().to_mesh(volumetric_base::Layer::SKIN);

    prism_adapter::transfer_to_surface_mesh(newSkel, blendshape, volumetric_base::Layer::SKEL);
    prism_adapter::transfer_to_surface_mesh(newMuscle, blendshape, volumetric_base::Layer::MUSC);
    prism_adapter::transfer_to_surface_mesh(newSkin, blendshape, volumetric_base::Layer::SKIN);

    fixer.reattach_body_parts(state, oldSkin, newSkin);
    fixer.transfer_fixes(state, newSkin, newMuscle, newSkel);
    skel.skinning_from_baked_mesh(newSkel);
    musc.skinning_from_baked_mesh(newMuscle);
    skin.skinning_from_baked_mesh(newSkin);
}

// ---------------------------------------------------------------------------------------------------------------------

void CorotationalLinearElasticSkinner::updated_input_layers(volumetric_base::VolumetricState* state)
{
    std::cout << "Updating input layers " << std::endl;

    if(!diffgeo_solver)
    {
        std::cerr << "Called mesh combination without having instantiated mean meshes. "<< std::endl;
        return;
    }
    _combined_a_pose_mesh= state->get_layers();
    volumetric_base::cleanMesh(_combined_a_pose_mesh);

    a_pose_diffgeo_solver.initialize(_combined_a_pose_mesh);
    if(!_animator)
        _animator = new PrismBasedCorotationalElasticity(_combined_a_pose_mesh);
    else
        _animator->setup_new_mesh(_combined_a_pose_mesh);
}

// ---------------------------------------------------------------------------------------------------------------------

void CorotationalLinearElasticSkinner::updated_mean_layers(volumetric_base::VolumetricState* state)
{
    std::cout << "Initialized mean " << std::endl;
    const volumetric_base::LayeredMesh mean = state->mean_mesh();

    delete diffgeo_solver;
    diffgeo_solver = new volumetric_base::diffgeo::DifferentialGeometrySolver(mean);
    diffgeo_solver->initialize(state);
}

// ---------------------------------------------------------------------------------------------------------------------

void CorotationalLinearElasticSkinner::updated_pose(volumetric_base::VolumetricState* state)
{
    std::cout << "Updating pose" << std::endl;
    if(_animator==nullptr)
    {
        std::cerr << "Called mesh combination without having instantiated mean meshes. "<< std::endl;
        exit(0);

    }
    _animated_mesh = _combined_a_pose_mesh;
    std::cout << "Entering Skinning" << std::endl;
    if(!_animator)
    {
        std::cerr << "Animator called before initializing" << std::endl;
        exit(3);
    }
    volumetric_skinning::basic_skinning::setup_mesh_properties_for_cleaned_mesh(_animated_mesh,  state->get_skinning_parameters(volumetric_base::Layer::SKEL));


}

// ---------------------------------------------------------------------------------------------------------------------

void CorotationalLinearElasticSkinner::imgui_update(volumetric_character::VolumetricState *state)
{
    SkinningBase::imgui_update(state);
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================
