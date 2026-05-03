// =====================================================================================================================
// Created by fabian on 9/19/24.
// mail@fabiankemper.de
// =====================================================================================================================

#pragma once
#ifndef SKINNING_BASE_H
#define SKINNING_BASE_H

// =====================================================================================================================

#include "volanim_utils/defs.h"
#include "volumetric_base/skinnable_mesh.h"
#include "volumetric_base/state/volumetric_state.h"


// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_skinning)

class SkinningBase {
public:
    SkinningBase();
    virtual ~SkinningBase();

    // set mean meshes for volumetric layers
    virtual auto updated_mean_layers(volumetric_base::VolumetricState* state) -> void;

    // set new input layers
    virtual auto updated_input_layers(volumetric_base::VolumetricState* state) -> void;

    // perform skinning, update gradient matrix
    virtual auto updated_pose(volumetric_base::VolumetricState* state) -> void;

    // convert internal deformation gradients into a final mesh
    // all SkinnableMeshes are writable parameters (so output should be written in these references)
    virtual auto perform_skinning(
        volumetric_base::VolumetricState* state,
        volumetric_base::SkinnableMesh& skel,
        volumetric_base::SkinnableMesh& musc,
        volumetric_base::SkinnableMesh& skin,
        bool converge = false
    ) -> void;

    // wake up from cache
    virtual auto wake_up() -> void;

    // show gui options, e.g. perform extra iteration
    virtual auto imgui_update(volumetric_base::VolumetricState *state) -> void;

    /// Show Corrective Skinning Blendshapes
    virtual auto  perform_blendshape_skinning(
        volumetric_base::VolumetricState * state,
        volumetric_base::SkinnableMesh& skel, volumetric_base::SkinnableMesh& musc, volumetric_base::SkinnableMesh& skin
    ) -> void;

    bool dirty = false;
};

// =====================================================================================================================

NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================

#endif //SKINNING_BASE_H

// =====================================================================================================================
