// =====================================================================================================================

#pragma once
#ifndef PMPCHARACTER_CHARACTER_RENDERER_H
#define PMPCHARACTER_CHARACTER_RENDERER_H

// =====================================================================================================================

#include "volanim_utils/defs.h"
#include "warped_mesh_renderer.h"

#include "volumetric_character/volumetric_character.h"
#include "volumetric_character/renderer/skeleton_renderer.h"
#include "volumetric_character/renderer/skinnable_mesh_renderer.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_character)

// =====================================================================================================================

class VolumetricCharacterRenderer
{
public:
    // constructor
    [[maybe_unused]] explicit
    VolumetricCharacterRenderer(VolumetricCharacter& volumetric_character);
    virtual ~VolumetricCharacterRenderer();

    // public members
    bool show_skeleton = false;

    // render calls
    auto draw(const pmp::mat4& model_view, const pmp::mat4& model_view_projection, const pmp::mat3& normal,
              const std::string& draw_mode) const -> void;

    // set alpha for layers
    auto set_alpha(float skeleton_alpha, float muscle_alpha, float skin_alpha) const -> void;
    auto set_highres_weights(const bary_warp::BarycentricWarp& warp) ->void;

    std::vector<bary_warp::WarpWeights> _hires_skeleton_warp_weights;
    std::vector<bary_warp::WarpWeights> _hires_muscle_warp_weights;
protected:
    VolumetricCharacter& _character;
    // renderer
    SkinnableMeshRenderer* _skel_wrap_renderer;
    SkinnableMeshRenderer* _musc_wrap_renderer;
    SkinnableMeshRenderer* _skin_renderer;

    // high-res renderer
    WarpedMeshRenderer* _highres_skel_renderer;
    WarpedMeshRenderer* _highres_musc_renderer;

    SkeletonRenderer* _skeleton_renderer;
    auto draw_skeleton(const pmp::mat4& model_view_projection) const -> void;
};


// =====================================================================================================================

NAMESPACE_END(volumetric_character)

// =====================================================================================================================

#endif //PMPCHARACTER_CHARACTER_RENDERER_H

// =====================================================================================================================
