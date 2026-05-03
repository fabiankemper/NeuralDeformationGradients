// =====================================================================================================================
// Created by fabian on 9/19/24.
// mail@fabiankemper.de
// =====================================================================================================================

#include "tet_volumetric.h"

#include "volumetric_base/state/utils.h"
#include "volumetric_base/state/skinning/skinning.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_skinning)

// =====================================================================================================================

using Layer = volumetric_base::Layer;

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

TetVolumetricSkinning::TetVolumetricSkinning() : LinearBlendSkinning()
{
}

// ---------------------------------------------------------------------------------------------------------------------

TetVolumetricSkinning::~TetVolumetricSkinning() = default;

// ---------------------------------------------------------------------------------------------------------------------

auto TetVolumetricSkinning::updated_pose(volumetric_base::VolumetricState* state) -> void
{
    if (state == nullptr) {
        return;
    }

    // perform LBS for skeleton layer
    volumetric_base::skinning::lbs(state, _result_skel, Layer::SKEL);
    volumetric_base::skinning::lbs(state, _result_musc, Layer::MUSC);
    volumetric_base::skinning::lbs(state, _result_skin, Layer::SKIN);

    // perform linear blend skinning for base skeleton
    auto skel = _result_skel.bake_mesh();
    auto musc = _result_musc.bake_mesh();
    auto skin = _result_skin.bake_mesh();

    // copy result layers as skinning to final meshes
    _result_skel.skinning_from_baked_mesh(skel);
    _result_musc.skinning_from_baked_mesh(musc);
    _result_skin.skinning_from_baked_mesh(skin);

    // apply head, hand, toe fix
    _fix_collapsed_vertices(state);
}


// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================
