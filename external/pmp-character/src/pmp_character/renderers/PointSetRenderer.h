//
// Created by peasman on 01.07.24.
//

#ifndef POINTSETRENDERER_H
#define POINTSETRENDERER_H

#include "Renderer.h"
#include <pmp/visualization/renderer.h>
#include "pmp_character/node_types/PointSet.h"


class PointSetRenderer : public pmp_character::Renderer {
    public:
        explicit PointSetRenderer(const std::shared_ptr<pmp_character::PointSet> &set);
        ~PointSetRenderer();

        std::shared_ptr<pmp_character::PointSet> set;
        float point_size = 10.0f;

        void draw(const pmp::mat4 &modelview, const pmp::mat4 &modelviewprojection, const pmp::mat3 &normal, std::string draw_mode) override;
    protected:
        void create_opengl_buffers();
        void delete_opengl_buffers();

        void update_opengl_buffers();


        GLuint vao_ = 0;
        GLuint vertex_buffer_ = 0;
        GLuint normal_buffer_ = 0;
        GLuint color_buffer_ = 0;

        pmp::Shader shader_;

        float ambient_   = 0.1;
        float diffuse_   = 0.8;
        float specular_  = 0.6;
        float shininess_ = 100.0;
        float alpha_     = 1.0;
};



#endif //POINTSETRENDERER_H
