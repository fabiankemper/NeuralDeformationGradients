//
// Created by peasman on 13.12.23.
//

#include "SkinnedMeshNodeRenderer.h"

#include "../PropertyNames.h"
#include "pmp/algorithms/normals.h"
#include "pmp/visualization/phong_shader.h"
#include "../node_types/Character.h"


namespace pmp_character
{

    SkinnedMeshNodeRenderer::SkinnedMeshNodeRenderer(std::shared_ptr<MeshNode> node) : Renderer(node)
    {
        tex_coord_buffer_ = 0;
        vertex_array_object_ = 0;
        vertex_buffer_ = 0;
        color_buffer_ = 0;
        normal_buffer_ = 0;
        tex_coord_buffer_ = 0;
        edge_buffer_ = 0;
        feature_buffer_ = 0;

        // initialize buffer sizes
        vertexBufferSize = 0;

        // material parameters
        material_.front_color_ = pmp::Point(0.6, 0.6, 0.6);
        material_.back_color_ = pmp::Point(0.5, 0.0, 0.0);
        material_.ambient_ = 0.1;
        material_.diffuse_ = 0.8;
        material_.specular_ = 0.2;
        material_.shininess_ = 15.0;
        material_.alpha_ = 1.0;
        material_.crease_angle_ = 180.0;
        material_.point_size_ = 5.0;

        // initialize texture
        texture_id_ = 0;
        show = true;
        mesh_node_ = node;
    }

    SkinnedMeshNodeRenderer::~SkinnedMeshNodeRenderer()
    {
        // delete OpenGL buffers
        glDeleteBuffers(1, &vertex_buffer_);
        glDeleteBuffers(1, &color_buffer_);
        glDeleteBuffers(1, &normal_buffer_);
        glDeleteBuffers(1, &tex_coord_buffer_);
        glDeleteBuffers(1, &edge_buffer_);
        glDeleteBuffers(1, &feature_buffer_);
        glDeleteVertexArrays(1, &vertex_array_object_);
    }
    void SkinnedMeshNodeRenderer::loadTexture(Character* character)
    {
        glEnable(GL_DEPTH_TEST);
        std::cout << "Loaded Texture" << std::endl;
        if (texture_id_)
        {
            return;
        }
        auto &currentTexture = character->textureAtlas[mesh_node_->texture_name];
        auto &textureName = mesh_node_->texture_name;
        if (textureIDs.find(textureName) == textureIDs.end())
        {

            // std::cout << "Found Texture in Atlas: " << textureName << std::endl;
            // Gen Texture
            glGenTextures(1, &texture_id_);
            textureIDs[textureName] = texture_id_;

            // Bind Texture
            glBindTexture(GL_TEXTURE_2D, texture_id_);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB, currentTexture.width, currentTexture.height, 0, GL_RGBA,
                         GL_UNSIGNED_BYTE, currentTexture.textureBuffer);

            // set texture properties
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

            // turn on mipmapping and anisotropic filtering when not in browser
#ifndef __EMSCRIPTEN__
            glGenerateMipmap(GL_TEXTURE_2D);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            // GLfloat aniso;
            // glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &aniso);
            // glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, aniso);
#endif
        }
        else
        {

            // std::cerr << "Texture already in atlas. Not loading" << textureName << std::endl;
            texture_id_ = textureIDs[textureName];
        }
        glDisable(GL_DEPTH_TEST);

    }
    void SkinnedMeshNodeRenderer::create_buffers()
    {
        if (mesh_node_->mesh.n_vertices() == 0)
        {
            std::cerr << "Mesh has no vertices or was not initialized." << std::endl;
        }
        if (!phong_shader.is_valid())
        {
            try
            {
                phong_shader.source(phong_vshader, phong_fshader);
            }
            catch (const pmp::GLException &e)
            {
                std::cerr << e.what() << std::endl;
                exit(1);
            }
        }
        // update_opengl_buffers();
        glGenVertexArrays(1, &vertex_array_object_);
        glBindVertexArray(vertex_array_object_);
        glGenBuffers(1, &vertex_buffer_);
        glGenBuffers(1, &color_buffer_);
        glGenBuffers(1, &normal_buffer_);
        glGenBuffers(1, &tex_coord_buffer_);
        glGenBuffers(1, &edge_buffer_);
        glGenBuffers(1, &feature_buffer_);
        pmp::SurfaceMesh &mesh_ = mesh_node_->mesh;

        // get properties
        auto vpos = mesh_.vertex_property<pmp::Point>(properties::vertexSkinnedPos);
        auto vcolor = mesh_.vertex_property<pmp::Color>(properties::vertexColor);
        auto vtex = mesh_.get_vertex_property<pmp::TexCoord>(properties::vertexTextureCoordinates);
        auto htex = mesh_.get_halfedge_property<pmp::TexCoord>(properties::halfedgeTextureCoordinates);
        auto fcolor = mesh_.get_face_property<pmp::Color>(properties::faceColor);

        // index array for remapping vertex indices during duplication
        std::vector<size_t> vertex_indices(mesh_.n_vertices());

        // produce arrays of points, normals, and texcoords
        // (duplicate vertices to allow for flat shading)
        std::vector<pmp::Point> position_array;
        std::vector<pmp::Point> color_array;
        std::vector<pmp::Point> normal_array;
        std::vector<pmp::vec2> tex_array;
        std::vector<pmp::ivec3> triangles;
        if (mesh_.n_faces())
        {
            // reserve memory
            position_array.reserve(3 * mesh_.n_faces());
            normal_array.reserve(3 * mesh_.n_faces());
            if (htex || vtex)
                tex_array.reserve(3 * mesh_.n_faces());

            if ((vcolor || fcolor) && use_colors_)
                color_array.reserve(3 * mesh_.n_faces());

            // precompute normals for easy cases
            std::vector<pmp::Normal> face_normals;
            std::vector<pmp::Normal> vertex_normals;
            face_normals.reserve(mesh_.n_faces());
            vertex_normals.reserve(mesh_.n_vertices());
            if (material_.crease_angle_ < 1)
            {
                for (auto f : mesh_.faces())
                    face_normals.emplace_back(face_normal(mesh_, f));
            }
            else if (material_.crease_angle_ > 170)
            {
                for (auto v : mesh_.vertices())
                    vertex_normals.emplace_back(vertex_normal(mesh_, v));
            }

            // data per face (for all corners)
            std::vector<pmp::Halfedge> corner_halfedges;
            std::vector<pmp::Vertex> corner_vertices;
            std::vector<pmp::Point> corner_positions;
            std::vector<pmp::Point> corner_colors;
            std::vector<pmp::Point> corner_normals;
            std::vector<pmp::vec2> corner_texcoords;

            // convert from degrees to radians
            const pmp::Scalar crease_angle_radians = material_.crease_angle_ / 180.0 * M_PI;

            size_t vidx(0);
            // loop over all faces
            for (auto f : mesh_.faces())
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
                for (auto h : mesh_.halfedges(f))
                {
                    v = mesh_.to_vertex(h);
                    corner_halfedges.push_back(h);
                    corner_vertices.push_back(v);
                    corner_positions.push_back(vpos[v]);

                    if (material_.crease_angle_ < 1)
                    {
                        n = face_normals[f.idx()];
                    }
                    else if (material_.crease_angle_ > 170)
                    {
                        n = vertex_normals[v.idx()];
                    }
                    else
                    {
                        n = corner_normal(mesh_, h, crease_angle_radians);
                    }
                    corner_normals.push_back((pmp::Point)n);

                    if (htex)
                    {
                        corner_texcoords.push_back((pmp::vec2)htex[h]);
                    }
                    else if (vtex)
                    {
                        corner_texcoords.push_back((pmp::vec2)vtex[v]);
                    }

                    if (vcolor && use_colors_)
                    {
                        corner_colors.push_back((pmp::Point)vcolor[v]);
                    }
                    else if (fcolor && use_colors_)
                    {
                        corner_colors.push_back((pmp::Point)fcolor[f]);
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
                if ((vcolor || fcolor) && use_colors_)
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
                glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
                glBufferData(GL_ARRAY_BUFFER, position_array.size() * 3 * sizeof(float), position_array.data(),
                             GL_STATIC_DRAW);
                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
                glEnableVertexAttribArray(0);
                vertexBufferSize = position_array.size();
            }
            else
            {
                glDisableVertexAttribArray(0);
                vertexBufferSize = 0;
            }

            // upload normals
            if (!normal_array.empty())
            {
                glBindBuffer(GL_ARRAY_BUFFER, normal_buffer_);
                glBufferData(GL_ARRAY_BUFFER, normal_array.size() * 3 * sizeof(float), normal_array.data(),
                             GL_STATIC_DRAW);
                glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
                glEnableVertexAttribArray(1);
            }
            else
            {
                glDisableVertexAttribArray(1);
            }
            if (mesh_.n_edges())
            {
                std::vector<unsigned int> edge_indices;
                edge_indices.reserve(mesh_.n_edges());

                for (auto e : mesh_.edges())
                {
                    auto v0 = mesh_.vertex(e, 0).idx();
                    auto v1 = mesh_.vertex(e, 1).idx();
                    edge_indices.push_back(vertex_indices[v0]);
                    edge_indices.push_back(vertex_indices[v1]);
                }

                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, edge_buffer_);
                glBufferData(GL_ELEMENT_ARRAY_BUFFER, edge_indices.size() * sizeof(unsigned int), edge_indices.data(),
                             GL_STATIC_DRAW);
            }
            if (!tex_array.empty())
            {
                glBindBuffer(GL_ARRAY_BUFFER, tex_coord_buffer_);
                glBufferData(GL_ARRAY_BUFFER, tex_array.size() * 2 * sizeof(float), tex_array.data(), GL_STATIC_DRAW);
                glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, nullptr);
                glEnableVertexAttribArray(2);
            }
            else
            {
                glDisableVertexAttribArray(2);
            }

            glBindVertexArray(0);

            // std::cout << "finished filling buffers" << std::endl;
        }
    }
    void SkinnedMeshNodeRenderer::update_skinned_vertices_and_normals()
    {
        if (not vertex_array_object_)
        {
            create_buffers();
        }

        std::vector<pmp::Point> points;
        points.reserve(mesh_node_->mesh.n_faces() * 3);

        auto sm_points = mesh_node_->mesh.get_vertex_property<pmp::Point>(properties::vertexGlobalPosition);
        for (auto f : mesh_node_->mesh.faces())
        {
            std::vector<pmp::Vertex> faceVertcies;
            for (auto v : mesh_node_->mesh.vertices(f))
            {
                faceVertcies.emplace_back(v);
            }
            points.push_back(sm_points[faceVertcies[0]]);
            points.push_back(sm_points[faceVertcies[1]]);
            points.push_back(sm_points[faceVertcies[2]]);
        }

        glBindVertexArray(vertex_array_object_);
        // vertices
        glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_);
        glBufferData(GL_ARRAY_BUFFER, points.size() * 3 * sizeof(float), points.data(), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(0);
        // TODO Add Normals
        vertexBufferSize = (unsigned int)points.size();

        std::vector<pmp::Point> normal_array;
        auto& mesh_ = mesh_node_->mesh;
        if (mesh_node_->mesh.n_faces())
        {
            // reserve memory
            normal_array.reserve(3 * mesh_.n_faces());
            // precompute normals for easy cases
            std::vector<pmp::Normal> face_normals;
            std::vector<pmp::Normal> vertex_normals;
            face_normals.reserve(mesh_.n_faces());
            vertex_normals.reserve(mesh_.n_vertices());
            if (material_.crease_angle_ < 1)
            {
                for (auto f : mesh_.faces())
                    face_normals.emplace_back(face_normal(mesh_, f));
            }
            else if (material_.crease_angle_ > 170)
            {
                for (auto v : mesh_.vertices())
                    vertex_normals.emplace_back(vertex_normal(mesh_, v));
            }

            // data per face (for all corners)
            std::vector<pmp::Halfedge> corner_halfedges;
            std::vector<pmp::Vertex> corner_vertices;
            std::vector<pmp::Point> corner_positions;
            std::vector<pmp::Point> corner_colors;
            std::vector<pmp::Point> corner_normals;
            std::vector<pmp::vec2> corner_texcoords;

            // convert from degrees to radians
            const pmp::Scalar crease_angle_radians = material_.crease_angle_ / 180.0 * M_PI;

            // loop over all faces
            for (auto f : mesh_.faces())
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
                for (auto h : mesh_.halfedges(f))
                {
                    v = mesh_.to_vertex(h);
                    corner_halfedges.push_back(h);
                    corner_vertices.push_back(v);

                    if (material_.crease_angle_ < 1)
                    {
                        n = face_normals[f.idx()];
                    }
                    else if (material_.crease_angle_ > 170)
                    {
                        n = vertex_normals[v.idx()];
                    }
                    else
                    {
                        n = corner_normal(mesh_, h, crease_angle_radians);
                    }
                    corner_normals.push_back((pmp::Point)n);
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
            glBindBuffer(GL_ARRAY_BUFFER, normal_buffer_);
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


        // create_buffers();
        // glBindVertexArray(vertex_array_object_);
    }
    void SkinnedMeshNodeRenderer::draw(const pmp::mat4 &modelview, const pmp::mat4 &modelviewprojection,
                                const pmp::mat3 &normal, std::string draw_mode)
    {
        if (!vertex_array_object_)
        {
            create_buffers();
        }
        glBindVertexArray(vertex_array_object_);
        glEnable(GL_DEPTH_TEST);
        if(not phong_shader.is_valid())
        {
            phong_shader.source(phong_vshader, phong_fshader);
        }
        phong_shader.use();
        phong_shader.set_uniform("modelview_projection_matrix", modelviewprojection);
        phong_shader.set_uniform("modelview_matrix", modelview);
        phong_shader.set_uniform("normal_matrix", normal);
        phong_shader.set_uniform("point_size", material_.point_size_);
        phong_shader.set_uniform("light1", pmp::vec3(1.0, 1.0, 1.0));
        phong_shader.set_uniform("light2", pmp::vec3(-1.0, 1.0, 1.0));
        phong_shader.set_uniform("front_color", pmp::vec3(material_.front_color_));
        phong_shader.set_uniform("back_color", pmp::vec3(material_.back_color_));
        phong_shader.set_uniform("ambient", material_.ambient_);
        phong_shader.set_uniform("diffuse", material_.diffuse_);
        phong_shader.set_uniform("specular", material_.specular_);
        phong_shader.set_uniform("shininess", material_.shininess_);
        phong_shader.set_uniform("alpha", material_.alpha_);
        phong_shader.set_uniform("use_lighting", true);

        phong_shader.set_uniform("show_texture_layout", false);
        phong_shader.set_uniform("use_vertex_color", use_colors_);
        if (draw_mode == "Smooth Shading")
        {
            phong_shader.set_uniform("use_texture", false);
            phong_shader.set_uniform("use_srgb", false);
            if (mesh_node_->mesh.n_faces())
            {
                glDrawArrays(GL_TRIANGLES, 0, vertexBufferSize);
            }
            glBindVertexArray(0);
            phong_shader.disable();
            check_gl_errors();
        }
        else if (draw_mode == "Texture")
        {
            phong_shader.set_uniform("front_color", pmp::vec3(0.9, 0.9, 0.9));
            phong_shader.set_uniform("back_color", pmp::vec3(0.3, 0.3, 0.3));
            phong_shader.set_uniform("use_texture", true);
            phong_shader.set_uniform("use_vertex_color", false);
            phong_shader.set_uniform("use_srgb", true);
            glBindTexture(GL_TEXTURE_2D, texture_id_);
            glDrawArrays(GL_TRIANGLES, 0, vertexBufferSize);
            glBindVertexArray(0);
            phong_shader.disable();
            check_gl_errors();
        }
        glDisable(GL_DEPTH_TEST);

    }


} // namespace pmp_character
