//
// Created by peasman on 13.12.23.
//

#ifndef MESHNODERENDERER_H
#define MESHNODERENDERER_H

#include <pmp/visualization/shader.h>

#include "../MeshNode.h"
#include "../node_types/Character.h"
#include "Renderer.h"

namespace pmp_character
{
    struct Material
    {
        pmp::Color front_color_, back_color_;
        float ambient_, diffuse_, specular_, shininess_, alpha_;
        float crease_angle_;
        float point_size_;
        std::filesystem::path texture_path_;
    };
    class SkinnedMeshNodeRenderer : public Renderer
    {
    public:
        SkinnedMeshNodeRenderer(std::shared_ptr<MeshNode> node);
        ~SkinnedMeshNodeRenderer() override;
        void loadTexture(Character* character);
        std::shared_ptr<MeshNode> mesh_node_;
        std::map<std::string, GLuint> textureIDs;
        GLuint texture_id_;
        bool show = true;

    private:
        GLuint vertex_array_object_;
        GLuint vertex_buffer_;
        GLuint color_buffer_;
        GLuint normal_buffer_;
        GLuint tex_coord_buffer_;
        GLuint edge_buffer_;
        GLuint feature_buffer_;
        // buffer sizes
        GLsizei vertexBufferSize;
        Material material_;
        pmp::Shader phong_shader;
        bool use_colors_ = false;

    public:
        void create_buffers();
        void update_skinned_vertices_and_normals();
        void draw(const pmp::mat4 &modelview, const pmp::mat4 &modelviewprojection, const pmp::mat3 &normal,
                  std::string draw_mode) override;
    };
} // namespace pmp_character


#endif // MESHNODERENDERER_H
