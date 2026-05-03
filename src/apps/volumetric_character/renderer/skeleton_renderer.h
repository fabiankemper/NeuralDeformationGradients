// =====================================================================================================================

#pragma once
#ifndef SKELETON_RENDERER_H
#define SKELETON_RENDERER_H

// =====================================================================================================================

#include <pmp/visualization/shader.h>

#include "volanim_utils/defs.h"

#include "volumetric_character/volumetric_character.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_character)

// =====================================================================================================================


class SkeletonRenderer {
public:
    explicit SkeletonRenderer(VolumetricCharacter* character);
    virtual ~SkeletonRenderer();

    auto draw(const pmp::mat4 &model_view_projection) -> void;

    auto update_buffers() -> void;
protected:
public: // TODO: MAKE NON_PUBLIC
    volumetric_character::VolumetricCharacter* _volumetric_character;
    pmp::Shader _shader;

    GLuint _line_vertex_array_object = 0;
    GLuint _joint_vertex_array_object = 0;
    GLuint _vertex_buffer = 0;
    GLuint _color_buffer = 0;
    GLuint _joint_color_buffer = 0;
    GLuint _joint_buffer = 0;
    GLsizei _vertex_buffer_size = 0;
    GLuint _skeleton_cs_color_buffer = 0;

    auto draw_selected_joint() -> void;
};

// =====================================================================================================================

NAMESPACE_END(volumetric_character)

// =====================================================================================================================

#endif //SKELETON_RENDERER_H

// =====================================================================================================================
