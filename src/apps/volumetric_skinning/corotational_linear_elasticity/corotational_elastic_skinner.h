// =====================================================================================================================
#pragma once
#ifndef ELASTIC_SKINNER_H
#define ELASTIC_SKINNER_H

// =====================================================================================================================

#include "../../volumetric_base/diffgeo/diffgeo.h"
#include "volumetric_skinning/base/skinning_base.h"
#include "volumetric_skinning/corotational_linear_elasticity/prism_based_corotational_elasticity.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_skinning)

// =====================================================================================================================

class CorotationalLinearElasticSkinner : public volumetric_skinning::SkinningBase
{
public:
    void perform_skinning(
        volumetric_base::VolumetricState* state,
        volumetric_base::SkinnableMesh& skel,
        volumetric_base::SkinnableMesh& musc,
        volumetric_base::SkinnableMesh& skin,
        bool converge
    ) override;
    void perform_blendshape_skinning(volumetric_base::VolumetricState *state, volumetric_base::SkinnableMesh &skinnable_mesh,volumetric_base::SkinnableMesh &skinnable_layer, volumetric_base::SkinnableMesh &skinnable_mesh1) override;
    void updated_input_layers(volumetric_base::VolumetricState* state) override;
    void updated_mean_layers(volumetric_base::VolumetricState* state) override;
    void updated_pose(volumetric_base::VolumetricState* state) override;
    void imgui_update(volumetric_base::VolumetricState *state) override;

    volumetric_base::LayeredMesh _combined_a_pose_mesh;
    volumetric_base::LayeredMesh _animated_mesh;
    volumetric_base::diffgeo::DifferentialGeometrySolver* diffgeo_solver = nullptr;
    volumetric_base::diffgeo::DifferentialGeometrySolver a_pose_diffgeo_solver;
    PrismBasedCorotationalElasticity* _animator  = nullptr;
};

// =====================================================================================================================

NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================

#endif //ELASTIC_SKINNER_H

// =====================================================================================================================
