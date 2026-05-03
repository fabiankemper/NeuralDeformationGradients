//
// Created by peasman on 14.12.23.
//

#include "NodeRenderer.h"
#include <pmp/bounding_box.h>
#include "pmp/algorithms/utilities.h"
#include "shaders.h"
#include "pmp_character/utility/MatrixUtility.h"


namespace pmp_character
{
    NodeRenderer::~NodeRenderer()
    {
        if (vertex_buffer_)
            glDeleteBuffers(1, &vertex_buffer_);
        if (color_buffer_)
            glDeleteBuffers(1, &color_buffer_);
        if (line_vertex_array_object)
            glDeleteVertexArrays(1, &line_vertex_array_object);
    }

    NodeRenderer::NodeRenderer(std::shared_ptr<SceneNode> node) : Renderer(node), node(node)

    {
        if (vertex_buffer_)
            glDeleteBuffers(1, &vertex_buffer_);
        if (color_buffer_)
            glDeleteBuffers(1, &color_buffer_);
        if (line_vertex_array_object)
            glDeleteVertexArrays(1, &line_vertex_array_object);
    }
     void NodeRenderer::draw(const pmp::mat4 &modelview, const pmp::mat4 &modelviewprojection, const pmp::mat3 &normal, std::string draw_mode)
     {
        if (!shader_.is_valid())
        {
            shader_.source(skeleton_vshader, skeleton_fshader);
        }
        if(not line_vertex_array_object)
        {
            update();
        }

        pmp::mat4 mvp = modelviewprojection * node->transform;

        glBindVertexArray(line_vertex_array_object);

        shader_.use();
        shader_.set_uniform("modelview_projection_matrix", mvp);

        glDisable(GL_DEPTH_TEST);
        glDrawArrays(GL_LINES, 0, vertex_buffer_size_);
        shader_.disable();

        glEnable(GL_DEPTH_TEST);

        glBindVertexArray(0);

    }
    void NodeRenderer::update()
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



        std::vector<pmp::vec3> position_array;
        std::vector<pmp::vec3> color_array;

        vertex_buffer_size_ = 6;

        position_array.reserve(vertex_buffer_size_);
        color_array.reserve(vertex_buffer_size_);


        float cs_size =0.1f;

        const static pmp::vec3 x_axis = pmp::vec3(1.0f, 0.0f, 0.0f);
        const static pmp::vec3 y_axis = pmp::vec3(0.0f, 1.0f, 0.0f);
        const static pmp::vec3 z_axis = pmp::vec3(0.0f, 0.0f, 1.0f);
        const static pmp::vec3 gray = pmp::vec3(0.3f, 0.3f, 0.3f);

        pmp::vec3 jp = MatrixUtility::getTranslationPart(node->transform);


        position_array.push_back(jp);
        position_array.push_back(jp + cs_size * pmp::linear_transform(node->transform, x_axis));
        position_array.push_back(jp);
        position_array.push_back(jp + cs_size * pmp::linear_transform(node->transform, y_axis));
        position_array.push_back(jp);
        position_array.push_back(jp + cs_size * pmp::linear_transform(node->transform, z_axis));

        color_array.push_back(x_axis);
        color_array.push_back(x_axis);
        color_array.push_back(y_axis);
        color_array.push_back(y_axis);
        color_array.push_back(z_axis);
        color_array.push_back(z_axis);


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
