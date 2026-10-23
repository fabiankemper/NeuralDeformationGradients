#ifndef FBX_FILE_H
#define FBX_FILE_H

#include <filesystem>
#include <string>
#include "../node_types/Character.h"

#include <pmp/surface_mesh.h>

// ufbx types, forward declared to keep ufbx.h out of this header
struct ufbx_node;
struct ufbx_matrix;
struct ufbx_skin_deformer;

namespace pmp_character
{


    /// \brief Reads characters (skeleton, meshes, skinning weights) from FBX files using ufbx.
    class FBX_file
    {
    private:
        std::string filename_;

        Character *character_;


    public:
        FBX_file();
        ~FBX_file();


        bool read(Character *character, std::filesystem::path filename);

        // helpers
    private:
        void traverse_nodes_rec(const ufbx_node *node, bool skeleton);

        void fbxskeleton_to_skeleton_rec(const ufbx_node *fbxnode, Skeleton &skeleton, Joint *parent);
        bool fbxmesh_to_surfacemesh(const ufbx_node *fbxnode, MeshNode *node);
        bool fbxskin_to_surfacemesh(const ufbx_skin_deformer *fbxskin, MeshNode *node);

        // other helpers
        static void fbxmat_to_mat(const ufbx_matrix &fbxmat, pmp::Mat4<pmp::Scalar> &mat);
    };


} // namespace pmp_character


#endif
