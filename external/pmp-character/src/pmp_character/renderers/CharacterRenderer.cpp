//
// Created by peasman on 13.12.23.
//

#include "CharacterRenderer.h"

#include "../stb_image/stb_image.h"

namespace pmp_character
{
    CharacterRenderer::CharacterRenderer(const std::shared_ptr<Character> &_character) : Renderer(_character),
                                                                                         draw_mode_("Smooth Shading"), vertex_array_(0), texcoord_buffer_(0), weight_buffer_(0), texture_id_(0), rainbow_texture_(0),
                                                                                         vertex_buffer_(0), color_buffer_(0), normal_buffer_(0)
    {
        character = _character;
        node = _character;
        skeletonRenderer = new SkeletonRenderer(*character);
        for (auto &child : character->children)
        {
            mesh_node_renderers.emplace_back(new SkinnedMeshNodeRenderer(std::dynamic_pointer_cast<MeshNode>(child)));
        }
    }

    CharacterRenderer::~CharacterRenderer()
    {
        if (texture_id_)
            glDeleteTextures(1, &texture_id_);
        if (rainbow_texture_)
            glDeleteTextures(1, &rainbow_texture_);
        if (vertex_buffer_)
            glDeleteBuffers(1, &vertex_buffer_);
        if (normal_buffer_)
            glDeleteBuffers(1, &normal_buffer_);
        if (texcoord_buffer_)
            glDeleteBuffers(1, &texcoord_buffer_);
        if (weight_buffer_)
            glDeleteBuffers(1, &weight_buffer_);
        if (vertex_array_)
            glDeleteVertexArrays(1, &vertex_array_);
    }

    void CharacterRenderer::load_textures()
    {
        for (auto &child : mesh_node_renderers)
        {
            if (child->texture_id_)
            {
                continue;
            }
            auto &currentTexture = character->textureAtlas[child->mesh_node_->texture_name];
            auto &textureName = child->mesh_node_->texture_name;
            if (textureIDs.find(textureName) == textureIDs.end())
            {

                // std::cout << "Found Texture in Atlas: " << textureName << std::endl;
                // Gen Texture
                glGenTextures(1, &child->texture_id_);
                textureIDs[textureName] = child->texture_id_;
                // Bind Texture
                glBindTexture(GL_TEXTURE_2D, child->texture_id_);
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
                child->texture_id_ = textureIDs[textureName];
            }
        }
    }
    // TODO Still on initial mesh which doesnt work
    void CharacterRenderer::select_joint(int joint)
    {
        // // parent's work
        character->skeleton_.select_joint(joint);
    }
    void CharacterRenderer::draw(const pmp::mat4& modelview, const pmp::mat4& modelviewprojection, const pmp::mat3& normal,
                                 std::string draw_mode)
    {
        if(draw_mode == "Texture" && not mesh_node_renderers[0]->texture_id_)
        {
            load_textures();
        }
        if(character->is_dirty())
        {
            // std::cout << "Updated vertices\n";
            character->update();
            skeletonRenderer->update();
            updateSkinnedVerticesAndBlendshapes();
            character->set_dirty(false);
        }
        for (auto &node : mesh_node_renderers)
        {
            if(node->show)
            {
                node->draw(modelview, modelviewprojection, normal, draw_mode);
            }
        }
        if(renderSkeleton)
        {
            draw_skeleton(modelviewprojection);
        }

    }

    void CharacterRenderer::updateSkinnedVerticesAndBlendshapes()
    {
        // std::cout << "Updating skinned vertices \n";
        for (auto &child : mesh_node_renderers)
        {
            child->update_skinned_vertices_and_normals();
        }
    }

    void CharacterRenderer::draw_skeleton(const pmp::mat4 &modelviewprojection)
    {
        skeletonRenderer->draw(modelviewprojection);
    }

} // namespace pmp_character
