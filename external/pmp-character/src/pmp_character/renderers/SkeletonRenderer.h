//
// Created by peasman on 14.12.23.
//

#ifndef PMPCHARACTER_SKELETONRENDERER_H
#define PMPCHARACTER_SKELETONRENDERER_H

#include <pmp/visualization/shader.h>
#include "../node_types/Character.h"
#include "../Skeleton.h"
#include "Renderer.h"

namespace pmp_character
{
    class SkeletonRenderer
    {
    public:
        SkeletonRenderer();

        ~SkeletonRenderer();

        explicit SkeletonRenderer(Character &character);
        void drawSelectedJoint();
        void draw(const pmp::mat4 &modelviewprojection);
        void update();

        Character *character_;
        pmp::Shader shader_;
        GLuint line_vertex_array_object;
        GLuint joint_vertex_array_object;
        GLuint vertex_buffer_;
        GLuint color_buffer_;
        GLuint joint_color_buffer_;
        GLuint joint_buffer_;
        GLsizei vertex_buffer_size_;
        GLuint skeleton_cs_color_buffer_ = 0;
    };
} // namespace pmp_character

#endif // PMPCHARACTER_SKELETONRENDERER_H
