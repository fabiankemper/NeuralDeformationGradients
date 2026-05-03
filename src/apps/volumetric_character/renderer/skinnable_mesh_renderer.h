// =====================================================================================================================

#pragma once
#ifndef VOLUMETRIC_ANIMATION_SKINNABLE_MESH_RENDERER_H
#define VOLUMETRIC_ANIMATION_SKINNABLE_MESH_RENDERER_H

// =====================================================================================================================

#include <pmp_character/renderers/SkinnedMeshNodeRenderer.h>

#include "volumetric_base/aliases.h"
#include "volumetric_base/skinnable_mesh.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_character)

// =====================================================================================================================

using Material = pmp_character::Material;
using SkinnableMesh = volumetric_base::SkinnableMesh;

// =====================================================================================================================

class SkinnableMeshRenderer
{
public:
    explicit SkinnableMeshRenderer(SkinnableMesh& skinnable_mesh);
    virtual ~SkinnableMeshRenderer();

    auto draw(
        const pmp::mat4& model_view, const pmp::mat4& model_view_projection,
        const pmp::mat3& normal, const std::string& draw_mode
    ) -> void;

    // getter and setter
    auto material() -> Material& { return _material; }
    [[nodiscard]]
    auto material() const -> const Material& { return _material; }

    // todo: why public?
    GLuint texture_id = 0;
protected:
    SkinnableMesh& _skinnable_mesh;
    Material _material;

    // OpenGL
    GLuint _vertex_array_object;
    GLuint _vertex_buffer;
    GLuint _color_buffer;
    GLuint _normal_buffer;
    GLuint _tex_coord_buffer;
    GLuint _edge_buffer;
    GLuint _feature_buffer;
    // buffer sizes
    GLsizei _vertexBufferSize;

    pmp::Shader _phong_shader;
    bool _use_colors = false;


    // upload buffers to openGL
    virtual void create_buffers();

    virtual void update_buffers();
};

// =====================================================================================================================

NAMESPACE_END(volumetric_character)

// =====================================================================================================================

#endif //VOLUMETRIC_ANIMATION_SKINNABLE_MESH_RENDERER_H

// =====================================================================================================================
