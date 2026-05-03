//
// Created by gerrit on 12.07.24.
//
// =====================================================================================================================

#pragma once
#ifndef VOLUMETRIC_FIXER_H
#define VOLUMETRIC_FIXER_H

// =====================================================================================================================

#include <pmp/surface_mesh.h>

#include "prism_adapter.h"
#include "volumetric_base/layered_mesh.h"
#include "volumetric_base/state/volumetric_state.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_skinning)

// =====================================================================================================================


// =====================================================================================================================

class VolumetricBoundaryFixer
{
public:
    explicit VolumetricBoundaryFixer(volumetric_base::LayeredMesh &mesh);
    auto reattach_body_parts(volumetric_base::VolumetricState* state, pmp::SurfaceMesh& origin, pmp::SurfaceMesh& target) -> void;

    static void transfer_fixes(const volumetric_base::VolumetricState* state, const pmp::SurfaceMesh& surface_mesh, pmp::SurfaceMesh& new_muscle, pmp::SurfaceMesh& new_skel);

    static pmp::SurfaceMesh linear_blend_skin(pmp::SurfaceMesh mesh, volumetric_base::Skeleton& skeleton,
                                              pmp::SurfaceMesh& propsource);

    static auto find_border_regions(volumetric_base::LayeredMesh& mesh) -> std::vector<std::vector<int> >;

    static volumetric_base::LayeredMesh fixed_layered_mesh(volumetric_base::LayeredMesh& cleaned, volumetric_base::VolumetricState* state ) {

        volumetric_skinning::VolumetricBoundaryFixer fixer(cleaned);

        pmp::SurfaceMesh oldSkin = state->get_layers().to_mesh(volumetric_base::Layer::SKIN);
        pmp::SurfaceMesh newSkel = state->get_layers().to_mesh(volumetric_base::Layer::SKEL);
        pmp::SurfaceMesh newMuscle = state->get_layers().to_mesh(volumetric_base::Layer::MUSC);
        pmp::SurfaceMesh newSkin = state->get_layers().to_mesh(volumetric_base::Layer::SKIN);

        prism_adapter::transfer_to_surface_mesh(newSkel, cleaned, volumetric_base::Layer::SKEL);
        prism_adapter::transfer_to_surface_mesh(newMuscle, cleaned, volumetric_base::Layer::MUSC);
        prism_adapter::transfer_to_surface_mesh(newSkin, cleaned, volumetric_base::Layer::SKIN);
        fixer.reattach_body_parts(state, oldSkin, newSkin);
        fixer.transfer_fixes(state, newSkin, newMuscle, newSkel);
        return {newSkel, newMuscle, newSkin};
    }
    std::vector<std::vector<int> > border_regions;
};

// =====================================================================================================================

NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================

#endif //VOLUMETRIC_FIXER_H

// =====================================================================================================================
