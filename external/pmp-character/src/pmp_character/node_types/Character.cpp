//=============================================================================
//
//   Exercise code for the lecture
//   "Computer Animation"
//   by Prof. Dr. Mario Botsch, TU Dortmund
//
//   Copyright (C)  Computer Graphics Group, TU Dortmund.
//
//=============================================================================

//== INCLUDES =================================================================
#define STB_IMAGE_IMPLEMENTATION

#include "Character.h"
#include "../DualQuaternion.h"

#include <float.h>

#include <pmp/algorithms/normals.h>

#include "../PropertyNames.h"
#include <pmp/io/io.h>


//== IMPLEMENTATION ===========================================================
namespace pmp_character
{
    //-----------------------------------------------------------------------------

    void Character::write_obj(const std::filesystem::path &filename)
    {
        auto &mesh = get_biggest_mesh()->mesh;
        // Read out the skinned/global positions into a new mesh that is going to be written
        pmp::SurfaceMesh output_mesh;
        output_mesh.assign(mesh);
        auto globalPosition = mesh.vertex_property<pmp::Point>(properties::vertexGlobalPosition);
        auto globalNormal = mesh.vertex_property<pmp::Normal>(properties::vertexGlobalNormal);
        auto normal = output_mesh.vertex_property<pmp::Normal>(properties::vertexNormals);
        for (auto v : mesh.vertices())
        {
            output_mesh.position(v) = globalPosition[v];
            normal[v] = globalNormal[v];
        }
        pmp::write(output_mesh, filename);
    }

    std::shared_ptr<MeshNode> Character::get_biggest_mesh()
    {
        size_t biggestSize = 0;
        int biggestIndex = -1;
        for (int i = 0; i < children.size(); i++)
        {
            auto meshNode = get_mesh_by_index(i);
            if (meshNode->mesh.n_vertices() > biggestSize)
            {
                biggestSize = meshNode->mesh.n_vertices();
                biggestIndex = i;
            }
        }
        return get_mesh_by_index(biggestIndex);
    }

    MeshNode &Character::get_mesh_by_name(std::string meshName)
    {
        for (auto child : children)
        {
            auto childMeshNode = std::dynamic_pointer_cast<MeshNode>(child);
            if (childMeshNode->name == meshName)
            {
                return *childMeshNode;
            }
        }
        throw MeshNotFoundException(meshName);
    }

    int Character::get_mesh_index_by_name(std::string meshName)
    {
        for (size_t i = 0; i < children.size(); i++)
        {
            auto child = get_mesh_by_index(i);
            if (child->name == meshName)
            {
                return i;
            }
        }
        throw MeshNotFoundException(meshName);
    }

    std::shared_ptr<MeshNode> Character::get_mesh_by_index(int index)
    {
        if (index < children.size())
        {
            return std::dynamic_pointer_cast<MeshNode>(children[index]);
        }
        std::cerr << "Mesh with index" << index << " not found. Aborting" << std::endl;
        exit(1);
    }

    std::vector<M4x4> &Character::getSkinningMatrices()
    {
        if (skinning_matrices_.empty())
        {
            skinning();
        }
        return skinning_matrices_;
    }

    void Character::load_texture(std::filesystem::path texture_path, MeshNode &mesh_node)
    {
        std::string texture_name = texture_path.filename().string();
        Texture texture;
        texture.textureBuffer = stbi_load(texture_path.string().c_str(), &texture.width, &texture.height, &texture.numComponents,
                                          4);
        texture.byteSize = texture.width * texture.height * texture.numComponents * sizeof(float);
        texture.filepath = texture_name;
        // Check if correctly loaded
        if (texture.width == 0 or texture.height == 0 or texture.numComponents == 0)
        {
            std::cerr << "Texture could not be loaded. Check for errors. Path: " << texture_path << std::endl;
        }
        textureAtlas[texture_name] = texture;
        mesh_node.texture_name = texture_name;
        set_dirty(true);
    }

    //-----------------------------------------------------------------------------

    void Character::apply_blendshape()
    {
        for (size_t i = 0; i < children.size(); i++)
        {
            auto currentMesh = get_mesh_by_index(i);
            auto &currentBlendshapes = currentMesh->blendshape_collection;
            if (currentBlendshapes.empty)
                continue;
            // Reset blendpos
            auto blendpos = currentMesh->mesh.vertex_property<pmp::Point>(properties::vertexBlendPosition);
            auto blend_normal = currentMesh->mesh.vertex_property<pmp::Point>(properties::vertexBlendNormal);
            auto normal = currentMesh->mesh.vertex_property<pmp::Point>(properties::vertexNormals);
            for (const pmp::Vertex &v : currentMesh->mesh.vertices())
            {
                blendpos[v] = currentMesh->mesh.position(v);
                blend_normal[v] = normal[v];
            }
            for (size_t j = 0; j < currentBlendshapes.blendshapes.size(); j++)
            {
                auto &currentBlendshape = currentBlendshapes.blendshapes[j];
                currentBlendshape.applyBlendshape(currentMesh->mesh, blendshapeEffectors[j]);
            }
        }
    }

    void Character::calculate_size_and_center()
    {
        pmp::Point min_(FLT_MAX), max_(-FLT_MAX);
        // Find skin mesh
        int biggestMesh = 0;
        for (size_t i = 0; i < children.size(); i++)
        {
            auto child = std::dynamic_pointer_cast<MeshNode>(children[i]);

            if (child->name.find("skin") != std::string::npos)
            {
                biggestMesh = i;
                break;
            }
            if (child->mesh.n_vertices() > std::dynamic_pointer_cast<MeshNode>(children[biggestMesh])->mesh.
                n_vertices())
            {
                biggestMesh = i;
            }
        }
        for (auto position : std::dynamic_pointer_cast<MeshNode>(children[biggestMesh])->mesh.positions())
        {
            for (int j = 0; j < 3; ++j)
            {
                if (position[j] < min_[j])
                    min_[j] = position[j];
                if (position[j] > max_[j])
                    max_[j] = position[j];
            }
        }
        skeleton_.center_ = (max_ + min_) * 0.5f;
        skeleton_.size_ = pmp::norm(max_ - min_);
    }

    //-----------------------------------------------------------------------------


    //-----------------------------------------------------------------------------

    Character::Character() : SceneNode("Character"), skinning_mode_(LINEAR_BLEND_SKINNING), dirty_(false)
    {
        inverse_kinematics.set_skeleton(&skeleton_);
        nodeType = NodeType::Character;
    }

    Character::Character(Character &other) : SceneNode(other.name),
                                             blendshapeEffectors(other.blendshapeEffectors),
                                             blendshapeNames(other.blendshapeNames),
                                             textureAtlas(other.textureAtlas),
                                             skinning_mode_(other.skinning_mode_)
    {
        skeleton_ = other.skeleton_;
        nodeType = NodeType::Character;
        inverse_kinematics.set_skeleton(&skeleton_);
        for (auto child : other.children)
        {
            auto childMeshNode = std::dynamic_pointer_cast<MeshNode>(child);
            auto newChildMeshNode = std::make_shared<MeshNode>(*childMeshNode);
            this->children.emplace_back(newChildMeshNode);
        }
    }

    Character::Character(const std::shared_ptr<Character> &other) : SceneNode(other->name),
                                                                    skeleton_(Skeleton(other->skeleton_)),
                                                                    blendshapeEffectors(other->blendshapeEffectors),
                                                                    blendshapeNames(other->blendshapeNames),
                                                                    textureAtlas(other->textureAtlas),
                                                                    skinning_mode_(other->skinning_mode_)
    {
        nodeType = NodeType::Character;
        inverse_kinematics.set_skeleton(&skeleton_);
        for (auto child : other->children)
        {
            auto childMeshNode = *std::dynamic_pointer_cast<MeshNode>(child);
            auto newChildMeshNode = std::make_shared<MeshNode>(childMeshNode);
            this->children.emplace_back(newChildMeshNode);
        }

    }

    void Character::set_skinning_mode(int mode)
    {
        if (mode != skinning_mode_)
        {
            skinning_mode_ = (SkinningMode)mode;
            skinning();
        }
    }

    //-----------------------------------------------------------------------------

    void Character::skinning()
    {
        // setup array of joint transformations, represented both as
        // 4x4 matrices and as 8D dual quaterions
        skinning_matrices_.resize(skeleton_.joints_.size());
        skinning_dualquats_.resize(skeleton_.joints_.size());
        for (unsigned int i = 0; i < skeleton_.joints_.size(); ++i)
        {
            skinning_matrices_[i] = skeleton_.joints_[i]->transformation_;
            skinning_dualquats_[i] = mat4_to_dualquat(skeleton_.joints_[i]->transformation_);
        }

        // now we compute skinning
        switch (skinning_mode_)
        {
        case LINEAR_BLEND_SKINNING:
            linear_blend_skinning();
            break;

        case DUAL_QUATERNION_SKINNING:
            dual_quaternion_skinning();
            break;

        default:
            break;
        }
        set_dirty(true);
    }

    //-----------------------------------------------------------------------------

    void Character::linear_blend_skinning()
    {
        // Use Skinning to manipulate all meshes
        for (auto &child : children)
        {
            auto childMeshNode = std::dynamic_pointer_cast<MeshNode>(child);
            auto skinned_vertices_ = childMeshNode->mesh.vertex_property<pmp::vec3>(properties::vertexSkinnedPos);
            auto global_pos = childMeshNode->mesh.vertex_property<pmp::vec3>(properties::vertexGlobalPosition);
            auto skinned_normals_ = childMeshNode->mesh.vertex_property<pmp::vec3>(properties::vertexSkinnedNormal);
            auto global_normal = childMeshNode->mesh.vertex_property<pmp::vec3>(properties::vertexGlobalNormal);

            // resize skinned vertex and normal vectors
            auto normals = childMeshNode->mesh.vertex_property<pmp::Normal>(properties::vertexNormals);
            pmp::VertexProperty<pmp::vec3> blendpos;
            if (childMeshNode->mesh.has_vertex_property(properties::vertexBlendPosition))
            {
                blendpos = childMeshNode->mesh.get_vertex_property<pmp::vec3>(properties::vertexBlendPosition);
            }
            else
            {
                blendpos = childMeshNode->mesh.get_vertex_property<pmp::vec3>(properties::vertexPosition);
            }
            auto v_depends = childMeshNode->mesh.get_vertex_property<ivec8>(properties::vertexSkinningDepends);
            auto v_weights = childMeshNode->mesh.get_vertex_property<vec8>(properties::vertexSkinningWeights);
            // auto blend_normal = child.mesh.get_vertex_property<pmp::vec3>(properties::vertexBlendNormal);
            // #pragma omp parallel for
            for (auto v : childMeshNode->mesh.vertices())
            {
                pmp::Vector<float, 3> currentVertex = childMeshNode->mesh.position(v);
                pmp::Vector<float, 3> currentNormal = normals[v];
                pmp::mat4 blend_matrix = pmp::mat4(0);
                for (int j = 0; j < 8; j++)
                {
                    blend_matrix += v_weights[v][j] * skinning_matrices_[v_depends[v][j]];
                }
                skinned_vertices_[v] = affine_transform(blend_matrix, currentVertex);
                skinned_normals_[v] = linear_transform(blend_matrix, currentNormal);
                global_pos[v] = affine_transform(transform, skinned_vertices_[v]);
                global_normal[v] = affine_transform(transform, skinned_normals_[v]);
            }
            // child.create_buffers();
        }


        // upload skinned vertices and normals to the GPU for rendering
        // upload_to_GPU(skinned_vertices_, skinned_normals_);
    }

    //-----------------------------------------------------------------------------

    void Character::dual_quaternion_skinning()
    {
        for (auto &child : children)
        {
            auto childMeshNode = std::dynamic_pointer_cast<MeshNode>(child);
            std::cout << "Doing DQS on " << childMeshNode->name << std::endl;
            auto skinned_vertices_ = childMeshNode->mesh.vertex_property<pmp::vec3>(properties::vertexSkinnedPos);
            auto global_pos = childMeshNode->mesh.vertex_property<pmp::vec3>(properties::vertexGlobalPosition);
            auto skinned_normals_ = childMeshNode->mesh.vertex_property<pmp::vec3>(properties::vertexSkinnedNormal);
            auto global_normal = childMeshNode->mesh.vertex_property<pmp::vec3>(properties::vertexGlobalNormal);
            pmp::VertexProperty<pmp::vec3> blendpos;
            if (childMeshNode->mesh.has_vertex_property(properties::vertexBlendPosition))
            {
                blendpos = childMeshNode->mesh.get_vertex_property<pmp::vec3>(properties::vertexBlendPosition);
            }
            else
            {
                blendpos = childMeshNode->mesh.get_vertex_property<pmp::vec3>(properties::vertexPosition);
            }
            auto normals = childMeshNode->mesh.get_vertex_property<pmp::Normal>(properties::vertexNormals);
            auto v_depends = childMeshNode->mesh.get_vertex_property<ivec8>(properties::vertexSkinningDepends);
            auto v_weights = childMeshNode->mesh.get_vertex_property<vec8>(properties::vertexSkinningWeights);
            // #pragma omp parallel for
            for (auto v : childMeshNode->mesh.vertices())
            {
                vec8 quartsum = vec8(0);
                // q1 und q2 sind die aktuelle und die neue skinnindualquat in der forschleife
                for (int j = 0; j < 8; j++)
                {
                    vec8 temp = skinning_dualquats_[v_depends[v][j]];
                    quartsum += (dot(get_first_quat(quartsum), get_first_quat(temp)) > 0)
                        ? temp * v_weights[v][j]
                        : -temp * v_weights[v][j];
                    // quartsum += v_weights_[i][j]  * skinning_dualquats_[v_depends_[i][j]];
                }
                // Normalsieren
                quartsum /= norm(get_first_quat(quartsum));

                pmp::Mat4<pmp::Scalar> transmatrix = dualquat_to_mat4(quartsum);
                skinned_vertices_[v] = affine_transform(transmatrix, blendpos[v]);
                skinned_normals_[v] = linear_transform(transmatrix, normals[v]);
                global_pos[v] = affine_transform(transform, skinned_vertices_[v]);
                global_normal[v] = affine_transform(transform, skinned_normals_[v]);
            }

            // upload skinned vertices and normal to GPU for rendering
        }
    }

    void Character::set_dirty(bool dirty)
    {
        dirty_.store(dirty);
    }

    bool Character::is_dirty()
    {
        return dirty_.load();
    }
} // namespace pmp_character
//=============================================================================
