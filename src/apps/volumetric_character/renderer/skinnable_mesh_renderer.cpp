// =====================================================================================================================
#include "skinnable_mesh_renderer.h"
#include <pmp_character/PropertyNames.h>
#include "pmp_ext/normals.h"
#include "shader/skin_shader.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_character)

// =====================================================================================================================

namespace properties = pmp_character::properties;

// =====================================================================================================================


// ---------------------------------------------------------------------------------------------------------------------

SkinnableMeshRenderer::SkinnableMeshRenderer(SkinnableMesh& skinnable_mesh) : _skinnable_mesh(skinnable_mesh)
{

    _tex_coord_buffer = 0;
    _vertex_array_object = 0;
    _vertex_buffer = 0;
    _color_buffer = 0;
    _normal_buffer = 0;
    _tex_coord_buffer = 0;
    _edge_buffer = 0;
    _feature_buffer = 0;

    // initialize buffer sizes
    _vertexBufferSize = 0;

    // material parameters
    _material.front_color_ = pmp::vec3(0.6, 0.6, 0.6);
    _material.back_color_ = pmp::vec3(0.5, 0.0, 0.0);
    _material.ambient_ = 0.45;
    _material.diffuse_ = 0.65;
    _material.specular_ = 0.00;
    _material.shininess_ = 1.0;
    _material.alpha_ = 1.0;
    _material.crease_angle_ = 180.0;
    _material.point_size_ = 5.0;

    // force an update buffers on first render
    _skinnable_mesh.set_dirty(true);
}

// ---------------------------------------------------------------------------------------------------------------------

SkinnableMeshRenderer::~SkinnableMeshRenderer()
{
    // delete OpenGL buffers
    glDeleteBuffers(1, &_vertex_buffer);
    glDeleteBuffers(1, &_color_buffer);
    glDeleteBuffers(1, &_normal_buffer);
    glDeleteBuffers(1, &_tex_coord_buffer);
    glDeleteBuffers(1, &_edge_buffer);
    glDeleteBuffers(1, &_feature_buffer);
    glDeleteVertexArrays(1, &_vertex_array_object);
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinnableMeshRenderer::draw(
    [[maybe_unused]] const pmp::mat4& model_view,
    const pmp::mat4& model_view_projection,
    const pmp::mat3& normal,
    const std::string& draw_mode
) -> void
{
    if (!_vertex_array_object) {
        create_buffers();
    }

    if (_skinnable_mesh.is_dirty()) {
        update_buffers();
    }

    const auto& mesh = _skinnable_mesh.mesh();

    glBindVertexArray(_vertex_array_object);

    glEnable(GL_DEPTH_TEST);

    // allow for transparent objects
    glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE);

    if(not _phong_shader.is_valid())
    {
        std::cerr << "Draw call: Phong shader is not valid. Abort." << std::endl;
        //_phong_shader.source(character_vshader, character_fshader);
    }
    _phong_shader.use();
    _phong_shader.set_uniform("modelview_projection_matrix", model_view_projection);
    _phong_shader.set_uniform("modelview_matrix", model_view);
    _phong_shader.set_uniform("normal_matrix", normal);
    _phong_shader.set_uniform("point_size", _material.point_size_);
    _phong_shader.set_uniform("light1", pmp::vec3(1.0, 1.0, 1.0));
    _phong_shader.set_uniform("light2", pmp::vec3(-1.0, 1.0, 1.0));
    _phong_shader.set_uniform("front_color", _material.front_color_);
    _phong_shader.set_uniform("back_color", _material.back_color_);
    _phong_shader.set_uniform("ambient", _material.ambient_);
    _phong_shader.set_uniform("diffuse", _material.diffuse_);
    _phong_shader.set_uniform("specular", _material.specular_);
    _phong_shader.set_uniform("shininess", _material.shininess_);
    _phong_shader.set_uniform("alpha", _material.alpha_);
    _phong_shader.set_uniform("use_lighting", 1);

    _phong_shader.set_uniform("use_vertex_color", static_cast<int>(_use_colors));
    if (draw_mode == "Smooth Shading")
    {
        _phong_shader.set_uniform("use_texture", 0);
        _phong_shader.set_uniform("use_srgb", 0);
        if (mesh.n_faces())
        {
            glDrawArrays(GL_TRIANGLES, 0, _vertexBufferSize);
        }
        glBindVertexArray(0);
        _phong_shader.disable();
        check_gl_errors();
    }
    else if (draw_mode == "Texture")
    {
        _phong_shader.set_uniform("front_color", pmp::vec3(0.9, 0.9, 0.9));
        _phong_shader.set_uniform("back_color", pmp::vec3(0.3, 0.3, 0.3));
        _phong_shader.set_uniform("use_texture", 1);
        _phong_shader.set_uniform("use_vertex_color", 0);
        _phong_shader.set_uniform("use_srgb", 1);
        glBindTexture(GL_TEXTURE_2D, texture_id);
        glDrawArrays(GL_TRIANGLES, 0, _vertexBufferSize);
        glBindVertexArray(0);
        _phong_shader.disable();
        check_gl_errors();
    }
    else if (draw_mode == "Hidden Line")
    {
        // overlay edges
        glDepthRange(0.0, 1.0);
        glDepthFunc(GL_LEQUAL);
        _phong_shader.set_uniform("front_color", pmp::vec3(0.1, 0.1, 0.1));
        _phong_shader.set_uniform("back_color", pmp::vec3(0.1, 0.1, 0.1));
        _phong_shader.set_uniform("use_lighting", 0);
        _phong_shader.set_uniform("use_vertex_color", 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _edge_buffer);
        glDrawElements(GL_LINES, mesh.n_edges() * 2, GL_UNSIGNED_INT, nullptr);
        glDepthFunc(GL_LESS);
    }

    // disable transparency (doesn't work well with imgui)
    glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);

    glDisable(GL_DEPTH_TEST);
}

// ---------------------------------------------------------------------------------------------------------------------

void SkinnableMeshRenderer::create_buffers()
{
    if (_skinnable_mesh.mesh().n_vertices() == 0)
    {
        std::cerr << "Mesh has no vertices or was not initialized." << std::endl;
    }
    if (!_phong_shader.is_valid())
    {
        try
        {
            _phong_shader.source(volumetric_base::shaders::phong_vshader, volumetric_base::shaders::phong_fshader);
        }
        catch (const pmp::GLException &e)
        {
            std::cerr << e.what() << std::endl;
            exit(1);
        }
    }
    // update_opengl_buffers();
    glGenVertexArrays(1, &_vertex_array_object);
    glBindVertexArray(_vertex_array_object);
    glGenBuffers(1, &_vertex_buffer);
    glGenBuffers(1, &_color_buffer);
    glGenBuffers(1, &_normal_buffer);
    glGenBuffers(1, &_tex_coord_buffer);
    glGenBuffers(1, &_edge_buffer);
    glGenBuffers(1, &_feature_buffer);

    pmp::SurfaceMesh const& mesh = _skinnable_mesh.mesh();

    // get properties
    auto vpos = mesh.get_vertex_property<pmp::Point>(properties::vertexSkinnedPos);
    auto vcolor = mesh.get_vertex_property<pmp::Color>(properties::vertexColor);
    auto vtex = mesh.get_vertex_property<pmp::TexCoord>(properties::vertexTextureCoordinates);
    auto htex = mesh.get_halfedge_property<pmp::TexCoord>(properties::halfedgeTextureCoordinates);
    auto fcolor = mesh.get_face_property<pmp::Color>(properties::faceColor);

    // index array for remapping vertex indices during duplication
    std::vector<size_t> vertex_indices(mesh.n_vertices());

    // produce arrays of points, normals, and texcoords
    // (duplicate vertices to allow for flat shading)
    std::vector<pmp::vec3> position_array;
    std::vector<pmp::vec3> color_array;
    std::vector<pmp::vec3> normal_array;
    std::vector<pmp::vec2> tex_array;
    std::vector<pmp::ivec3> triangles;
    if (mesh.n_faces() > 0)
    {
        // reserve memory
        position_array.reserve(3 * mesh.n_faces());
        normal_array.reserve(3 * mesh.n_faces());
        if (htex || vtex) {
            tex_array.reserve(3 * mesh.n_faces());
        }

        if ((vcolor || fcolor) && _use_colors) {
            color_array.reserve(3 * mesh.n_faces());
        }

        // precompute normals for easy cases
        std::vector<pmp::Normal> face_normals;
        std::vector<pmp::Normal> vertex_normals;
        face_normals.reserve(mesh.n_faces());
        vertex_normals.reserve(mesh.n_vertices());
        if (_material.crease_angle_ < 1)
        {
            for (auto f : mesh.faces()) {
                face_normals.emplace_back(pmp_ext::normals::face_normal(mesh, f, properties::vertexGlobalPosition));
            }
        }
        else if (_material.crease_angle_ > 170.0F)
        {
            for (auto v : mesh.vertices()) {
                vertex_normals.emplace_back(pmp_ext::normals::vertex_normal(mesh, v, properties::vertexGlobalPosition));
            }
        }

        // data per face (for all corners)
        std::vector<pmp::Halfedge> corner_halfedges;
        std::vector<pmp::Vertex> corner_vertices;
        std::vector<pmp::vec3> corner_positions;
        std::vector<pmp::vec3> corner_colors;
        std::vector<pmp::vec3> corner_normals;
        std::vector<pmp::vec2> corner_texcoords;

        // convert from degrees to radians
        const auto crease_angle_radians = static_cast<pmp::Scalar> (_material.crease_angle_ / 180.0 * M_PI);

        size_t vidx(0);
        // loop over all faces
        for (auto f : mesh.faces())
        {
            // collect corner positions and normals
            corner_halfedges.clear();
            corner_vertices.clear();
            corner_positions.clear();
            corner_colors.clear();
            corner_normals.clear();
            corner_texcoords.clear();
            pmp::Vertex v;
            pmp::Normal n;
            std::vector<int> vertexIndices;
            for (auto h : mesh.halfedges(f))
            {
                v = mesh.to_vertex(h);
                corner_halfedges.push_back(h);
                corner_vertices.push_back(v);
                corner_positions.push_back(vpos[v]);

                if (_material.crease_angle_ < 1)
                {
                    n = face_normals[f.idx()];
                }
                else if (_material.crease_angle_ > 170.0F)
                {
                    n = vertex_normals[v.idx()];
                }
                else
                {
                    n = corner_normal(mesh, h, crease_angle_radians);
                }
                corner_normals.push_back((pmp::vec3)n);

                if (htex)
                {
                    corner_texcoords.push_back((pmp::vec2)htex[h]);
                }
                else if (vtex)
                {
                    corner_texcoords.push_back((pmp::vec2)vtex[v]);
                }

                if (vcolor && _use_colors)
                {
                    corner_colors.push_back((pmp::vec3)vcolor[v]);
                }
                else if (fcolor && _use_colors)
                {
                    corner_colors.push_back((pmp::vec3)fcolor[f]);
                }
            }
            for (auto vertex : corner_vertices)
            {
                vertexIndices.emplace_back(vertex.idx());
            }
            if (vertexIndices.size() > 3)
            {
                std::cerr << "Polygon with more than 3 vertices detected! Not supported!" << std::endl;
            }

            // triangles.emplace_back(vertexIndices[0], vertexIndices[1], vertexIndices[2]);
            triangles.emplace_back(0, 1, 2);
            pmp::ivec3 triangle(0, 1, 2);
            assert(corner_vertices.size() >= 3);

            // Get the indices of the triangle vertices
            int i0 = triangle[0];
            int i1 = triangle[1];
            int i2 = triangle[2];

            // Put the position of the vertex into the buffer array
            // position_array.push_back(initialMesh.position(Vertex(i0)));
            // position_array.push_back(initialMesh.position(Vertex(i1)));
            // position_array.push_back(initialMesh.position(Vertex(i2)));
            // position_array.push_back(corner_positions[i1]);
            // position_array.push_back(corner_positions[i2]);
            position_array.push_back(corner_positions[i0]);
            position_array.push_back(corner_positions[i1]);
            position_array.push_back(corner_positions[i2]);
            // Put the position of the vertex into the buffer array
            normal_array.push_back(corner_normals[i0]);
            normal_array.push_back(corner_normals[i1]);
            normal_array.push_back(corner_normals[i2]);

            // Put the texcoords into the buffer
            if (htex || vtex)
            {
                tex_array.push_back(corner_texcoords[i0]);
                tex_array.push_back(corner_texcoords[i1]);
                tex_array.push_back(corner_texcoords[i2]);
            }
            // Put the colors into the buffer
            if ((vcolor || fcolor) && _use_colors)
            {
                color_array.push_back(corner_colors[i0]);
                color_array.push_back(corner_colors[i1]);
                color_array.push_back(corner_colors[i2]);
            }
            // Put the vertex indices into the buffer
            vertex_indices[corner_vertices[i0].idx()] = vidx++;
            vertex_indices[corner_vertices[i1].idx()] = vidx++;
            vertex_indices[corner_vertices[i2].idx()] = vidx++;
        }

        // tessellate face into triangles
        // pmp::tesselate(corner_positions, triangles);

        // Upload all Buffers to the GPU
        // upload vertices
        if (!position_array.empty())
        {
            glBindBuffer(GL_ARRAY_BUFFER, _vertex_buffer);
            glBufferData(GL_ARRAY_BUFFER, position_array.size() * 3 * sizeof(float), position_array.data(),
                         GL_STATIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
            glEnableVertexAttribArray(0);
            _vertexBufferSize = position_array.size();
        }
        else
        {
            glDisableVertexAttribArray(0);
            _vertexBufferSize = 0;
        }

        // upload normals
        if (!normal_array.empty())
        {
            glBindBuffer(GL_ARRAY_BUFFER, _normal_buffer);
            glBufferData(GL_ARRAY_BUFFER, normal_array.size() * 3 * sizeof(float), normal_array.data(),
                         GL_STATIC_DRAW);
            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
            glEnableVertexAttribArray(1);
        }
        else
        {
            glDisableVertexAttribArray(1);
        }
        if (mesh.n_edges())
        {
            std::vector<unsigned int> edge_indices;
            edge_indices.reserve(mesh.n_edges());

            for (auto e : mesh.edges())
            {
                auto v0 = mesh.vertex(e, 0).idx();
                auto v1 = mesh.vertex(e, 1).idx();
                edge_indices.push_back(vertex_indices[v0]);
                edge_indices.push_back(vertex_indices[v1]);
            }

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _edge_buffer);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, edge_indices.size() * sizeof(unsigned int), edge_indices.data(),
                         GL_STATIC_DRAW);
        }
        if (!tex_array.empty())
        {
            glBindBuffer(GL_ARRAY_BUFFER, _tex_coord_buffer);
            glBufferData(GL_ARRAY_BUFFER, tex_array.size() * 2 * sizeof(float), tex_array.data(), GL_STATIC_DRAW);
            glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
            glEnableVertexAttribArray(2);
        }
        else
        {
            glDisableVertexAttribArray(2);
        }

        glBindVertexArray(0);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

void SkinnableMeshRenderer::update_buffers()
{
    auto& mesh = _skinnable_mesh.mesh();

    if (not mesh.is_triangle_mesh()) {
        std::cerr << "MeshNodeRenderer: Mesh is not pure triangle mesh. Could not render.";
        return;
    }

    std::vector<pmp::Point> points;
    points.reserve(mesh.n_faces() * 3);

    auto sm_points = mesh.get_vertex_property<pmp::vec3>(properties::vertexGlobalPosition);
    for (auto f : mesh.faces())
    {
        for (auto v : mesh.vertices(f))
        {
            points.push_back(sm_points[v]);
        }
    }

    glBindVertexArray(_vertex_array_object);
    // vertices
    glBindBuffer(GL_ARRAY_BUFFER, _vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, points.size() * 3 * sizeof(float), points.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(0);
    // TODO Add Normals
    _vertexBufferSize = static_cast<GLint> (points.size());

    std::vector<pmp::vec3> normal_array;

    if (mesh.n_faces())
    {
        // reserve memory
        normal_array.reserve(3 * mesh.n_faces());
        // precompute normals for easy cases
        std::vector<pmp::Normal> face_normals;
        std::vector<pmp::Normal> vertex_normals;
        face_normals.reserve(mesh.n_faces());
        vertex_normals.reserve(mesh.n_vertices());
        if (_material.crease_angle_ < 1)
        {
            for (auto f : mesh.faces()) {
                face_normals.emplace_back(pmp_ext::normals::face_normal(mesh, f, properties::vertexGlobalPosition));
            }
        }
        else if (_material.crease_angle_ > 170.0F)
        {
            for (auto v : mesh.vertices()) {
                vertex_normals.emplace_back(pmp_ext::normals::vertex_normal(mesh, v, properties::vertexGlobalPosition));
            }
        }

        // data per face (for all corners)
        std::vector<pmp::Halfedge> corner_halfedges;
        std::vector<pmp::Vertex> corner_vertices;
        std::vector<pmp::vec3> corner_positions;
        std::vector<pmp::vec3> corner_colors;
        std::vector<pmp::vec3> corner_normals;
        std::vector<pmp::vec2> corner_texcoords;

        // convert from degrees to radians
        const auto crease_angle_radians = static_cast<pmp::Scalar>(_material.crease_angle_ / 180.0 * M_PI);

        // loop over all faces
        for (auto f : mesh.faces())
        {
            // collect corner positions and normals
            corner_halfedges.clear();
            corner_vertices.clear();
            corner_positions.clear();
            corner_colors.clear();
            corner_normals.clear();
            corner_texcoords.clear();
            pmp::Vertex v;
            pmp::Normal n;
            std::vector<int> vertexIndices;
            for (auto h : mesh.halfedges(f))
            {
                v = mesh.to_vertex(h);
                corner_halfedges.push_back(h);
                corner_vertices.push_back(v);

                if (_material.crease_angle_ < 1)
                {
                    n = face_normals[f.idx()];
                }
                else if (_material.crease_angle_ > 170.0F)
                {
                    n = vertex_normals[v.idx()];
                }
                else
                {
                    n = corner_normal(mesh, h, crease_angle_radians);
                }
                corner_normals.push_back((pmp::vec3)n);
            }
            for (auto vertex : corner_vertices)
            {
                vertexIndices.emplace_back(vertex.idx());
            }
            if (vertexIndices.size() > 3)
            {
                std::cerr << "Polygon with more than 3 vertices detected! Not supported!" << std::endl;
            }

            // triangles.emplace_back(vertexIndices[0], vertexIndices[1], vertexIndices[2]);
            pmp::ivec3 triangle(0, 1, 2);
            assert(corner_vertices.size() >= 3);

            // Get the indices of the triangle vertices
            int i0 = triangle[0];
            int i1 = triangle[1];
            int i2 = triangle[2];

            // Put the position of the vertex into the buffer array
            normal_array.push_back(corner_normals[i0]);
            normal_array.push_back(corner_normals[i1]);
            normal_array.push_back(corner_normals[i2]);
        }
    }
    // upload normals
    if (!normal_array.empty())
    {
        glBindBuffer(GL_ARRAY_BUFFER, _normal_buffer);
        glBufferData(GL_ARRAY_BUFFER, normal_array.size() * 3 * sizeof(float), normal_array.data(),
                     GL_STATIC_DRAW);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glEnableVertexAttribArray(1);
    }
    else
    {
        glDisableVertexAttribArray(1);
    }

    glBindVertexArray(0);

    _skinnable_mesh.set_dirty(false);
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(volumetric_character::)

// =====================================================================================================================
