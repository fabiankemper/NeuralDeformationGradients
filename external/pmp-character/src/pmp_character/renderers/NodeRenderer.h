//
// Created by peasman on 28.06.24.
//

#ifndef NODERENDERER_H
#define NODERENDERER_H
#include "pmp_character/node_types/SceneNode.h"
#include <pmp/visualization/gl.h>
#include <pmp/visualization/shader.h>

#include "Renderer.h"

namespace pmp_character
{
    class NodeRenderer : public Renderer {
    public:
        ~NodeRenderer() override;
        explicit NodeRenderer(std::shared_ptr<SceneNode> node);
        void draw(const pmp::mat4 &modelview, const pmp::mat4 &modelviewprojection, const pmp::mat3 &normal, std::string draw_mode) override;
        void update() override;

        std::shared_ptr<SceneNode> node;

        pmp::Shader shader_;
        GLuint line_vertex_array_object =0;
        GLuint joint_vertex_array_object=0;
        GLuint vertex_buffer_=0;
        GLuint color_buffer_=0;
        GLuint joint_color_buffer_=0;
        GLuint joint_buffer_=0;
        GLsizei vertex_buffer_size_=0;
        GLuint skeleton_cs_color_buffer_ = 0;
    };
}

#endif //NODERENDERER_H
