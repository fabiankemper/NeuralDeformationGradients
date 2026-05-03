// =====================================================================================================================
// Created by fabian on 9/19/24.
// mail@fabiankemper.de
// =====================================================================================================================

#pragma once
#ifndef SKINNING_LBS_H
#define SKINNING_LBS_H

// =====================================================================================================================

#include "volanim_utils/defs.h"
#include "volumetric_character/volumetric_character.h"


#include "volumetric_skinning/base/skinning_base.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_skinning)

// =====================================================================================================================


class LinearBlendSkinning : public SkinningBase {
public:
    LinearBlendSkinning();
    ~LinearBlendSkinning() override;

    // updated mean meshes
    auto updated_mean_layers(volumetric_base::VolumetricState* state) -> void override;

    // updated layers
    auto updated_input_layers(volumetric_base::VolumetricState* state) -> void override;

    // updated pose
    auto updated_pose(volumetric_base::VolumetricState* state) -> void override;

    // generate final meshes
    auto perform_skinning(
        volumetric_base::VolumetricState* state,
        volumetric_base::SkinnableMesh& skel,
        volumetric_base::SkinnableMesh& musc,
        volumetric_base::SkinnableMesh& skin,
        bool converge
    ) -> void override;

protected:


    volumetric_character::DiffgeoSolver diffgeo_solver;
    // cached result (source)
    volumetric_base::SkinnableMesh _result_skel;
    volumetric_base::SkinnableMesh _result_musc;
    volumetric_base::SkinnableMesh _result_skin;

    // fix collapsed vertices
    auto _fix_collapsed_vertices(const volumetric_base::VolumetricState* state) -> void;
};

// =====================================================================================================================

NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================

#endif //SKINNING_LBS_H

// =====================================================================================================================
