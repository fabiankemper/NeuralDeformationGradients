#include "fbx_file.h"
#include <pmp/algorithms/normals.h>
#include <pmp/surface_mesh.h>
#include <ufbx.h>
#include <iostream>
#include <map>
#include "../PropertyNames.h"

using namespace pmp;
namespace pmp_character
{

    FBX_file::FBX_file() {}

    FBX_file::~FBX_file() {}

    bool FBX_file::read(Character *character, std::filesystem::path filename)
    {
        if (not exists(filename)) {
            throw std::runtime_error("FBX-File not found: " + filename.string());
        }
        filename_ = filename.string();
        character_ = character;

        // Convert to OpenGL coordinate system (right handed, y-up), keep the file's units
        ufbx_load_opts opts = {};
        opts.target_axes = ufbx_axes_right_handed_y_up;

        ufbx_error error;
        ufbx_scene *scene = ufbx_load_file(filename_.c_str(), &opts, &error);
        if (scene == nullptr)
        {
            std::cerr << "FBX_file::read: [ERROR] Could not load " << filename_ << ": " << error.description.data
                      << std::endl;
            return false;
        }

        // traverse for skeleton first, since we need it to properly build skinned mesh
        traverse_nodes_rec(scene->root_node, true);

        traverse_nodes_rec(scene->root_node, false);

        ufbx_free_scene(scene);

        character->skeleton_.joints_ = character_->skeleton_.joints_;
        for (auto &child : character->children)
        {
            auto childMeshNode = *dynamic_pointer_cast<MeshNode>(child);
            auto skinned_vertices = childMeshNode.mesh.add_vertex_property<pmp::vec3>(properties::vertexSkinnedPos);
            auto skinned_normal = childMeshNode.mesh.add_vertex_property<pmp::vec3>(properties::vertexSkinnedNormal);
            auto normals = childMeshNode.mesh.get_vertex_property<pmp::vec3>(properties::vertexNormals);
            for (auto v : childMeshNode.mesh.vertices())
            {
                skinned_vertices[v] = childMeshNode.mesh.position(v);
                skinned_normal[v] = normals[v];
            }
        }
        // Calculate Size
        character->calculate_size_and_center();

        // init kinematics, if skeleton given in fbx
        if (character_->skeleton_.root_) {
            character->skeleton_.root_->initialize_kinematics();
        }

        return true;
    }

    void FBX_file::traverse_nodes_rec(const ufbx_node *fbxnode, bool skeleton)
    {
        if (!skeleton && fbxnode->mesh != nullptr)
        {
            MeshNode *skin = new MeshNode();
            if (fbxmesh_to_surfacemesh(fbxnode, skin))
            {
                character_->children.emplace_back(skin);
            }
            else
            {
                delete skin;
            }
        }
        else if (skeleton && fbxnode->bone != nullptr)
        {
            fbxskeleton_to_skeleton_rec(fbxnode, character_->skeleton_, nullptr);
            return; // do not go further down the hierarchy, since fbxskeleton_to_skeleton_rec() does this
        }

        for (const ufbx_node *child : fbxnode->children)
        {
            traverse_nodes_rec(child, skeleton);
        }
    }

    void FBX_file::fbxskeleton_to_skeleton_rec(const ufbx_node *fbxnode, Skeleton &skeleton, Joint *parent)
    {
        if (fbxnode->bone == nullptr)
            return;

        // the root joint stores its global transform, all others the transform relative to their parent
        pmp::Mat4<Scalar> local;
        fbxmat_to_mat(parent == nullptr ? fbxnode->node_to_world : fbxnode->node_to_parent, local);

        Joint *current = new Joint(fbxnode->name.data, local);
        current->id = static_cast<int>(skeleton.joints_.size());
        skeleton.joints_.push_back(current);

        if (parent == nullptr)
        {
            skeleton.root_ = current;
        }
        else
        {
            current->parent_ = parent;
            parent->children_.push_back(current);
        }

        for (const ufbx_node *child : fbxnode->children)
        {
            fbxskeleton_to_skeleton_rec(child, skeleton, current);
        }
    }

    bool FBX_file::fbxmesh_to_surfacemesh(const ufbx_node *fbxnode, MeshNode *node)
    {
        const ufbx_mesh *fbxmesh = fbxnode->mesh;

        node->name = fbxnode->name.data;

        node->mesh.reserve(fbxmesh->num_vertices, fbxmesh->num_vertices * 3, fbxmesh->num_vertices / 3);

        Mat4<Scalar> transform;
        fbxmat_to_mat(fbxnode->node_to_world, transform);
        Mat3<Scalar> normal_transform = linear_part(transpose(inverse(transform)));
        node->transform = transform;

        // vertices are the FBX control points, baked into world space
        for (const ufbx_vec3 &p : fbxmesh->vertices)
        {
            node->mesh.add_vertex(affine_transform(transform, Point(p.x, p.y, p.z)));
        }

        std::vector<Vertex> idcs;
        for (const ufbx_face &face : fbxmesh->faces)
        {
            if (face.num_indices != 3)
            {
                std::cerr << "FBX_file::fbxmesh_to_surfacemesh: [ERROR] Mesh is not a triangle mesh." << std::endl;
                return false;
            }

            idcs.clear();
            for (uint32_t t = 0; t < face.num_indices; ++t)
            {
                idcs.emplace_back(Vertex(fbxmesh->vertex_indices[face.index_begin + t]));
            }
            node->mesh.add_face(idcs);
        }

        // normals are stored per face corner, average them per vertex
        auto normals = node->mesh.vertex_property<Normal>(properties::vertexNormals);
        if (fbxmesh->vertex_normal.exists)
        {
            for (auto v : node->mesh.vertices())
            {
                normals[v] = Normal(0, 0, 0);
            }
            for (size_t i = 0; i < fbxmesh->num_indices; ++i)
            {
                const ufbx_vec3 n = fbxmesh->vertex_normal[i];
                normals[Vertex(fbxmesh->vertex_indices[i])] += Normal(n.x, n.y, n.z);
            }
            for (auto v : node->mesh.vertices())
            {
                normals[v] = normalize(Normal(normal_transform * normals[v]));
            }
        }
        else
        {
            vertex_normals(node->mesh);
        }

        // texture coordinates and colors can only be stored, if they are unique per vertex (no seams)
        if (fbxmesh->vertex_uv.exists && fbxmesh->vertex_uv.unique_per_vertex)
        {
            auto texCoords = node->mesh.vertex_property<TexCoord>(properties::vertexTextureCoordinates);
            for (size_t i = 0; i < fbxmesh->num_indices; ++i)
            {
                const ufbx_vec2 uv = fbxmesh->vertex_uv[i];
                texCoords[Vertex(fbxmesh->vertex_indices[i])] = TexCoord(uv.x, uv.y);
            }
        }

        if (fbxmesh->vertex_color.exists && fbxmesh->vertex_color.unique_per_vertex)
        {
            auto colors = node->mesh.vertex_property<Color>(properties::vertexColor);
            for (size_t i = 0; i < fbxmesh->num_indices; ++i)
            {
                const ufbx_vec4 color = fbxmesh->vertex_color[i];
                colors[Vertex(fbxmesh->vertex_indices[i])] = Color(color.x, color.y, color.z);
            }
        }

        for (const ufbx_skin_deformer *fbxskin : fbxmesh->skin_deformers)
        {
            if (!fbxskin_to_surfacemesh(fbxskin, node))
            {
                return false;
            }
        }

        return true;
    }

    bool FBX_file::fbxskin_to_surfacemesh(const ufbx_skin_deformer *fbxskin, MeshNode *node)
    {
        std::vector<std::map<int, float>> dependWeights(node->mesh.n_vertices());

        for (const ufbx_skin_cluster *fbxcluster : fbxskin->clusters)
        {
            if (fbxcluster->bone_node == nullptr)
            {
                continue;
            }

            const int d = character_->skeleton_.get_joint_idx(fbxcluster->bone_node->name.data);

            if (d == -1)
            {
                std::cout << "Could not map FbxCluster with link name \"" << fbxcluster->bone_node->name.data << "\"."
                          << std::endl;
                continue;
            }

            for (size_t idx = 0; idx < fbxcluster->num_weights; ++idx)
            {
                dependWeights[fbxcluster->vertices[idx]][d] = (float)fbxcluster->weights[idx];
            }
        }

        VertexProperty<ivec8> depend1 = node->mesh.vertex_property<ivec8>(properties::vertexSkinningDepends);
        VertexProperty<vec8> weight1 = node->mesh.vertex_property<vec8>(properties::vertexSkinningWeights);

        for (size_t i = 0; i < dependWeights.size(); ++i)
        {
            const std::map<int, float> &m = dependWeights[i];
            ivec8 &dv1 = depend1.vector()[i];
            vec8 &wv1 = weight1.vector()[i];

            dv1 = ivec8(0.0f);
            wv1 = vec8(0);
            if (m.size() > 8)
            {
                std::cerr << "[ERROR] More than 8 bones for vertex: " << m.size()
                          << ". This is not supported currently." << std::endl;
                return false;
            }

            float w_sum = 0.0f;
            int c = 0;
            for (const auto &[d, w] : m)
            {
                w_sum += w;
                dv1[c] = (float)d;
                wv1[c] = w;
                ++c;
            }

            const float w_sum_inv = 1.0f / w_sum;
            wv1 *= w_sum_inv;
        }

        return true;
    }

    void FBX_file::fbxmat_to_mat(const ufbx_matrix &fbxmat, pmp::Mat4<Scalar> &mat)
    {
        // ufbx stores an affine 3x4 matrix as four columns
        for (int j = 0; j < 4; ++j)
        {
            for (int i = 0; i < 3; ++i)
            {
                mat(i, j) = fbxmat.cols[j].v[i];
            }
            mat(3, j) = j == 3 ? 1 : 0;
        }
    }

} // namespace pmp_character
