// =====================================================================================================================
//
// Created by gerrit-nolte on 8/19/24.
//
// =====================================================================================================================

#pragma once
#ifndef BASICSKINNING_H
#define BASICSKINNING_H

// =====================================================================================================================

#include <vector>

#include "volumetric_base/layered_mesh.h"
#include "volumetric_base/state/volumetric_state.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_skinning)
NAMESPACE_BEGIN(basic_skinning)

// =====================================================================================================================


// =====================================================================================================================

auto dual_quat_skin_skeleton(const volumetric_base::LayeredMesh& mesh,
                             const volumetric_base::VolumetricState* state) -> void;

auto linear_blend_skin_skeleton(volumetric_base::LayeredMesh& deformable_mesh,
                                const volumetric_base::VolumetricState* state) -> void;

auto linear_blend_skinning_matrices(volumetric_base::LayeredMesh &deformable_mesh,
    const volumetric_base::VolumetricState*state) -> std::vector<Eigen::Matrix4d>;

auto inverse_linear_blend_skin(volumetric_base::LayeredMesh &mesh, volumetric_base::VolumetricState const * state) -> void;

auto inverse_linear_blend_skin_safe(volumetric_base::LayeredMesh &mesh, volumetric_base::VolumetricState const * state) -> double;


auto linear_blend_skin(volumetric_base::LayeredMesh& mesh, volumetric_base::VolumetricState const* state) -> void;

auto setup_mesh_properties_for_cleaned_mesh(volumetric_base::LayeredMesh& mesh, const pmp::SurfaceMesh& propSource) -> void;

auto setup_mesh_properties_for_uncleaned_mesh(volumetric_base::LayeredMesh& mesh, const pmp::SurfaceMesh& propSource) -> void;

auto setup_mesh_properties_for_cleaned_mesh(pmp::SurfaceMesh &mesh, const volumetric_base::SkinningParameters &params) ->void;

auto setup_mesh_properties_for_uncleaned_mesh(pmp::SurfaceMesh &mesh, const volumetric_base::SkinningParameters &params) ->void;

// =====================================================================================================================

NAMESPACE_END(basic_skinning)

NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================

#endif //BASICSKINNING_H

// =====================================================================================================================
