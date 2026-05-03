//=============================================================================
//
//   Exercise code for the lecture
//   "Computer Animation"
//   by Prof. Dr. Mario Botsch, TU Dortmund
//
//   Copyright (C)  Computer Graphics Group, TU Dortmund.
//
//=============================================================================
#pragma once
//=============================================================================

#include "../Skeleton.h"

#include <vector>

#include <pmp/mat_vec.h>
#include <pmp/surface_mesh.h>
#include <pmp/algorithms/normals.h>

#include "../InverseKinematics.h"
#include "../stb_image/stb_image.h"

#include "../MeshNode.h"
#include "../Types.h"

namespace pmp_character
{
    struct Texture
    {
        stbi_uc *textureBuffer;
        std::string name;
        std::filesystem::path filepath;
        size_t byteSize;
        int width, height, numComponents;
    };
    class MeshNotFoundException : std::exception
    {
    public:
        explicit MeshNotFoundException(const std::string &meshName)
        {
            std::cerr << "Mesh with Name: " << meshName << " not found\n";
        }
    };

    //== CLASS DEFINITION =========================================================

    /** Class for representing a virtual character and computing skinning.
        The Character class inherits the Skeleton class, i.e., it also has an
        embedded skeleton and can trigger computation of forward kinematics.
        Its main functionality is to compute either linear blend skinning or dual
        quaternion skinning and to visualize the animated character mesh.
        \sa Skeleton
    */

    class Character : public SceneNode
    {
    public:
        /// constructor
        Character();
        Character(Character& other);
        Character(const std::shared_ptr<Character>& other);
        //
        // /// destructor
        ~Character() override = default;

        Skeleton skeleton_;
        Inverse_kinematics inverse_kinematics;
        std::vector<float> blendshapeEffectors;
        std::vector<std::string> blendshapeNames;
        std::map<std::string, Texture> textureAtlas;
        /// different skining methods
        enum SkinningMode
        {
            LINEAR_BLEND_SKINNING = 0,
            DUAL_QUATERNION_SKINNING = 1
        };
        /// set skinning method
        void set_skinning_mode(int mode);
        /// get skinning method
        int get_skinning_mode() { return (int)skinning_mode_; }


        void calculate_size_and_center();

        /// compute skinning using linear blend skinning or dual quaternion skinning
        void skinning();
        /// overloaded from Skeleton: computes forward kinematics and skinning
        void update() override
        {
            for(auto &child : children)
            {
                pmp::face_normals(std::dynamic_pointer_cast<MeshNode>(child)->mesh);
            }
            skeleton_.forward_kinematics();
            skinning();
            set_dirty((true));
        }

        /// write current skinned character to obj file
        void write_obj(const std::filesystem::path& filename);
        std::shared_ptr<MeshNode> get_biggest_mesh();
        MeshNode &get_mesh_by_name(std::string meshName);
        int get_mesh_index_by_name(std::string meshName);
        std::vector<M4x4> &getSkinningMatrices();
        std::shared_ptr<MeshNode> get_mesh_by_index(int index);
        void load_texture(std::filesystem::path texture_path, MeshNode& mesh_node);
        void apply_blendshape();
        void set_dirty(bool dirty);
        bool is_dirty();
    protected:
        /// load texture from file (called from load())

        /// compute linear blend skinning
        void linear_blend_skinning();
        /// compute dual quaternion skinning
        void dual_quaternion_skinning();
        /// which skinning mode has been selected
        SkinningMode skinning_mode_;
        /// skinning matrices for linear blend skinning
        std::vector<pmp::Mat4<pmp::Scalar>> skinning_matrices_;
        /// skinning dual quaternions for dual quaternion skinning
        std::vector<vec8> skinning_dualquats_;

        std::atomic<bool> dirty_;

    };
} // namespace pmp_character
//=============================================================================
