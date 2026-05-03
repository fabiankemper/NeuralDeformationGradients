//
// Created by peasman on 13.12.23.
//

#ifndef CHARACTERRENDERER_H
#define CHARACTERRENDERER_H
#include "../node_types/Character.h"
#include "SkinnedMeshNodeRenderer.h"
#include "SkeletonRenderer.h"


namespace pmp_character
{
    /// \brief Character Renderer class which provides all OpenGL flags to be used in rendering.
    class CharacterRenderer : public Renderer
    {
    public:
        CharacterRenderer(const std::shared_ptr<Character> &_character);
        ~CharacterRenderer();
        std::shared_ptr<Character> character;
        std::vector<std::shared_ptr<SkinnedMeshNodeRenderer>> mesh_node_renderers;
        /// Texture atlas
        std::map<std::string, GLuint> textureIDs;
        SkeletonRenderer *skeletonRenderer;
        // OpenGL stuff
        std::string draw_mode_;
        bool renderSkinOnly;
        bool renderSkeleton = true;
    private:
        pmp::Shader shader_;
        GLuint vertex_array_;
        GLuint texcoord_buffer_;
        GLuint weight_buffer_;
        GLuint texture_id_;
        GLuint rainbow_texture_;
        GLuint vertex_buffer_;
        GLuint color_buffer_;
        GLuint normal_buffer_;
        pmp::Shader phong_shader_;


    public:
        /// set drawing method
        void set_draw_mode(std::string mode) { draw_mode_ = mode; }
        /// get drawing mode
        std::string get_draw_mode() { return draw_mode_; }
        void load_textures();

        void select_joint(int joint);
        /// draw this character
        void draw(const pmp::mat4 &modelview, const pmp::mat4 &modelviewprojection, const pmp::mat3 &normal, std::string draw_mode) override;
        void draw_skeleton(const pmp::mat4 &modelviewprojection);
        void updateSkinnedVerticesAndBlendshapes();
    };
} // namespace pmp_character


#endif // CHARACTERRENDERER_H
