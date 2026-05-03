#include "PointSetRenderer.h"

#include "PointSetShader.h"

// TODO(swenninger): Namespacing

PointSetRenderer::PointSetRenderer(const std::shared_ptr<pmp_character::PointSet> &set) : Renderer(set),
                                                                                         set(set)
{

}

PointSetRenderer::~PointSetRenderer()
{
    delete_opengl_buffers();
}

void PointSetRenderer::create_opengl_buffers()
{
    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    glGenBuffers(1, &vertex_buffer_);
    glGenBuffers(1, &normal_buffer_);
    glGenBuffers(1, &color_buffer_);
}

void PointSetRenderer::delete_opengl_buffers()
{
    glDeleteBuffers(1, &color_buffer_);
    glDeleteBuffers(1, &normal_buffer_);
    glDeleteBuffers(1, &vertex_buffer_);

    glDeleteVertexArrays(1, &vao_);
}

void PointSetRenderer::update_opengl_buffers()
{
    if (!vao_)
    {
        create_opengl_buffers();
    }

    glBindVertexArray(vao_);

    glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);

    // Upload vertex positions
    if (!set->points.empty())
    {
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
        glBufferData(GL_ARRAY_BUFFER, set->points.size() * sizeof(set->points[0]),
                     set->points.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(0);
    }
    else
    {
        glDisableVertexAttribArray(0);
    }

    // Upload vertex normals
    if (set->normals.size() > 0)
    {
        glBindBuffer(GL_ARRAY_BUFFER, normal_buffer_);
        glBufferData(GL_ARRAY_BUFFER, set->normals.size() * sizeof(set->normals[0]),
                     set->normals.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(1);

    }
    else
    {
        glDisableVertexAttribArray(1);
    }

    // Upload vertex colors
    if (set->colors.size() > 0)
    {
        glBindBuffer(GL_ARRAY_BUFFER, color_buffer_);
        glBufferData(GL_ARRAY_BUFFER, set->colors.size() * sizeof(set->colors[0]),
                     set->colors.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(2);
    }
    else
    {
        glDisableVertexAttribArray(2);
    }

}

void
PointSetRenderer::draw(const pmp::mat4& modelview, const pmp::mat4& modelviewprojection,
                       const pmp::mat3& normal, std::string draw_mode)
{
    if (set->points.size() == 0)
    {
        return;
    }

    if (!vao_ || set->is_dirty())
    {
        update_opengl_buffers();
    }

    if (!shader_.is_valid())
    {
        //try
        {
            shader_.source(point_set_vshader, point_set_fshader);
        }
    }
    // setup matrices
    pmp::mat3 n_matrix = inverse(transpose(linear_part(modelview)));

    glBindVertexArray(vao_);

    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
    glEnable(GL_DEPTH_TEST);
    shader_.use();
    shader_.set_uniform("modelview_projection_matrix", modelviewprojection);
    shader_.set_uniform("modelview_matrix", modelview);
    shader_.set_uniform("normal_matrix", n_matrix);
    shader_.set_uniform("point_size", point_size);
    shader_.set_uniform("light1", pmp::vec3(1.0, 1.0, 1.0));
    shader_.set_uniform("light2", pmp::vec3(-1.0, 1.0, 1.0));
    shader_.set_uniform("ambient", ambient_);
    shader_.set_uniform("diffuse", diffuse_);
    shader_.set_uniform("specular", specular_);
    shader_.set_uniform("shininess", shininess_);
    shader_.set_uniform("use_lighting", false);
    shader_.set_uniform("use_srgb", false);

    glDrawArrays(GL_POINTS, 0, set->points.size());

    glDisable(GL_VERTEX_PROGRAM_POINT_SIZE);
    glDisable(GL_DEPTH_TEST);

    shader_.disable();
    glBindVertexArray(0);
}
