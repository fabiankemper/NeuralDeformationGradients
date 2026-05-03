//
// Created by peasman on 14.12.23.
//

#include "SkeletonRenderer.h"
#include <pmp/bounding_box.h>
#include "pmp/algorithms/utilities.h"
#include "shaders.h"


namespace pmp_character
{
    pmp_character::SkeletonRenderer::SkeletonRenderer() : line_vertex_array_object(0), vertex_buffer_(0), color_buffer_(0)
    {
        shader_.source(skeleton_vshader, skeleton_fshader);
    }

    SkeletonRenderer::~SkeletonRenderer()
    {
        if (vertex_buffer_)
            glDeleteBuffers(1, &vertex_buffer_);
        if (color_buffer_)
            glDeleteBuffers(1, &color_buffer_);
        if (line_vertex_array_object)
            glDeleteVertexArrays(1, &line_vertex_array_object);
    }

    SkeletonRenderer::SkeletonRenderer(Character &character) :
        line_vertex_array_object(0), vertex_buffer_(0), color_buffer_(0)
    {
        character_ = &character;
    }

    void SkeletonRenderer::drawSelectedJoint()
    {
        glBindVertexArray(joint_vertex_array_object);
        if(not shader_.is_valid())
        {
            shader_.source(skeleton_vshader, skeleton_fshader);
        }
        glDisable(GL_DEPTH_TEST);

        shader_.use();
        std::vector<pmp::Point> position_array;
        std::vector<pmp::Point> color_array;

        const pmp::Point x_ax = pmp::Point(1.0f, 0.0f, 0.0f);
        const pmp::Point y_ax = pmp::Point(0.0f, 1.0f, 0.0f);
        //const pmp::Point z_ax = pmp::Point(0.0f, 0.0f, 1.0f);

        // collect effectors (in red color)
        for (auto j : character_->skeleton_.joints_)
        {
            if (j->is_effector_)
            {
                position_array.push_back(j->get_global_translation());
                color_array.push_back(x_ax);
            }
        }

        // add selected joint (in green color)
        if (character_->skeleton_.selected_joint_ > 0 &&
            character_->skeleton_.selected_joint_ < (int)character_->skeleton_.joints_.size())
        {
            position_array.push_back(
                affine_transform(character_->transform, character_->skeleton_.joints_[character_->skeleton_.selected_joint_]->get_global_translation()));
            color_array.push_back(y_ax);
        }
        // upload buffer and draw as points
        if (not position_array.empty())
        {
            glBindBuffer(GL_ARRAY_BUFFER, joint_buffer_);
            glBufferData(GL_ARRAY_BUFFER, position_array.size() * sizeof(pmp::Point), &position_array[0], GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
            glEnableVertexAttribArray(0);
            glBindBuffer(GL_ARRAY_BUFFER, joint_color_buffer_);
            glBufferData(GL_ARRAY_BUFFER, color_array.size() * sizeof(pmp::Point), &color_array[0], GL_DYNAMIC_DRAW);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);
            glEnableVertexAttribArray(1);

        #ifndef __EMSCRIPTEN__
            glEnable(GL_PROGRAM_POINT_SIZE);
        #endif
            glDrawArrays(GL_POINTS, 0, position_array.size());
        }
        shader_.disable();

        glEnable(GL_DEPTH_TEST);
    }

    void SkeletonRenderer::draw(const pmp::mat4 &modelviewprojection)
    {
        if (character_->skeleton_.joints_.empty())
            return;
        if (!shader_.is_valid())
        {
            shader_.source(skeleton_vshader, skeleton_fshader);
        }
        if(not line_vertex_array_object)
        {
            update();
        }

        pmp::mat4 mvp = modelviewprojection;

        glBindVertexArray(line_vertex_array_object);

        shader_.use();
        shader_.set_uniform("modelview_projection_matrix", mvp);

        const bool gl_depth_test_enabled = glIsEnabled(GL_DEPTH_TEST);
        glDisable(GL_DEPTH_TEST);

        glDrawArrays(GL_LINES, 0, vertex_buffer_size_);
        shader_.disable();

        if (gl_depth_test_enabled) {
            glEnable(GL_DEPTH_TEST);
        }

        drawSelectedJoint();

        glBindVertexArray(0);
    }

    void SkeletonRenderer::update()
    {
        if(not line_vertex_array_object)
        {
            glGenVertexArrays(1, &line_vertex_array_object);
            glBindVertexArray(line_vertex_array_object);
            glGenBuffers(1, &color_buffer_);
            glGenBuffers(1, &skeleton_cs_color_buffer_);
            glGenVertexArrays(1,&joint_vertex_array_object);
            glGenBuffers(1, &joint_buffer_);
            glGenBuffers(1, &joint_color_buffer_);
        }


        std::vector<Joint*>& joints = character_->skeleton_.joints_;
        size_t num_joints = joints.size();

        std::vector<pmp::Point> position_array;
        std::vector<pmp::Point> color_array;

        vertex_buffer_size_ = 6 * num_joints;

        for (Joint* j : joints)
        {
            if (j->parent_)
            {
                vertex_buffer_size_ += 2;
            }
        }

        position_array.reserve(vertex_buffer_size_);
        color_array.reserve(vertex_buffer_size_);

        pmp::BoundingBox bb;
        for (auto node : character_->children)
        {
            auto meshNode = std::dynamic_pointer_cast<MeshNode>(node);
            bb += pmp::bounds(meshNode->mesh);
        }

        float cs_size = bb.size() * 0.01f;

        const static pmp::Point x_axis = pmp::Point(1.0f, 0.0f, 0.0f);
        const static pmp::Point y_axis = pmp::Point(0.0f, 1.0f, 0.0f);
        const static pmp::Point z_axis = pmp::Point(0.0f, 0.0f, 1.0f);
        const static pmp::Point gray = pmp::Point(0.3f, 0.3f, 0.3f);

        for (size_t joint_index = 0; joint_index < num_joints; ++joint_index)
        {
            Joint* j = joints[joint_index];
            pmp::Point jp = affine_transform(character_->transform, j->get_global_translation());

            if (j->parent_)
            {
                position_array.push_back(jp);
                // position_array.push_back(j->parent_->get_global_translation());
                position_array.push_back(
                                affine_transform(character_->transform, j->parent_->get_global_translation()));
                color_array.push_back(gray);
                color_array.push_back(gray);
            }

            position_array.push_back(jp);
            position_array.push_back(jp + cs_size * pmp::linear_transform(j->global_, x_axis));
            position_array.push_back(jp);
            position_array.push_back(jp + cs_size * pmp::linear_transform(j->global_, y_axis));
            position_array.push_back(jp);
            position_array.push_back(jp + cs_size * pmp::linear_transform(j->global_, z_axis));

            color_array.push_back(x_axis);
            color_array.push_back(x_axis);
            color_array.push_back(y_axis);
            color_array.push_back(y_axis);
            color_array.push_back(z_axis);
            color_array.push_back(z_axis);
        }

        glBindVertexArray(line_vertex_array_object);

        glBindBuffer(GL_ARRAY_BUFFER, color_buffer_);
        glBufferData(GL_ARRAY_BUFFER, position_array.size() * sizeof(position_array[0]),
                     position_array.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);

        glBindBuffer(GL_ARRAY_BUFFER, skeleton_cs_color_buffer_);
        glBufferData(GL_ARRAY_BUFFER, color_array.size() * sizeof(color_array[0]),
                     color_array.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(1);

        glBindVertexArray(0);





    }
} // namespace pmp_character
