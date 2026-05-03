#ifndef FBX_FILE_H
#define FBX_FILE_H

// Include fbxsdk.h while silencing the warnings it throws
#include "fbx_warnings.h"
FBX_DISABLE_WARNINGS_BEGIN
#include <fbxsdk.h>
FBX_DISABLE_WARNINGS_END

#include <map>
#include <set>
#include <string>
#include "../node_types/Character.h"

#include <pmp/surface_mesh.h>

namespace pmp_character
{


    class FBX_file
    {
    private:
        std::string filename_;

        Character *character_;

        FbxArray<FbxNode *> fbxnode_array_;

        FbxScene *fbxscene_;

        std::map<std::string, FbxFileTexture *> fbxtextures_;
        std::vector<pmp::SurfaceMesh> mesh_parts;
        std::map<std::string, pmp::SurfaceMesh> named_mesh_parts;


    public:
        FBX_file();
        ~FBX_file();


        bool write(const Character *character, const char *filename);

        bool read(Character *character, std::filesystem::path filename);

        // helpers
    private:
        // helpers for reading
        void traverse_nodes_rec(FbxNode *node, FbxNodeAttribute::EType type);

        void fbxskeleton_to_skeleton_rec(FbxNode *fbxnode, Skeleton &skeleton, Joint *parent);
        bool fbxmesh_to_surfacemesh(FbxNode *fbxnode, MeshNode *node);
        bool fbxskin_to_surfacemesh(FbxSkin *fbxskin, MeshNode *node);
        // bool fbxblendshape_to_blendshape(FbxBlendShape* fbxblendshape, BlendshapeCollection* blendshapes, const
        // Mat4f& transform = Mat4f::identity());
        void get_material(FbxNode *fbxnode, MeshNode *node);

        // helpers for writing
        bool skeleton_to_fbx_skeleton_rec(const Joint *joint, FbxNode *fbxnode);
        // bool surface_mesh_skin_to_fbxskin(const Surface_mesh_skin* skin, const Character *character);
        // Add the specified node to the node array. Also, add recursively
        // all the parent node of the specified node to the array.
        void add_node_rec(FbxNode *node);
        // FbxSurfacePhong* set_phong_material(FbxNode* fbxmeshnode, const Surface_mesh_skin* skin, const std::string&
        // meshid);
        void set_fbxtextures(FbxSurfacePhong *fbxmaterial, const pmp::SurfaceMesh *skin);


        // other helpers
        void fbxmat_to_mat(const FbxAMatrix &fbxmat, pmp::Mat4<pmp::Scalar>& mat);
        void mat_to_fbxmat(const pmp::Mat4<pmp::Scalar>& mat, FbxAMatrix &fbxmat);

        std::string extract_filepath(const std::string &filename);
        std::string extract_filename(const std::string &filename);
    };


} // namespace pmp_character


#endif
