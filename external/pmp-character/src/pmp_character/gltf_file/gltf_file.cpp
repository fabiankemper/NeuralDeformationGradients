#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "gltf_file.h"
#include <iostream>

#include <pmp/io/write_obj.h>
#include <pmp/stop_watch.h>
#include "../PropertyNames.h"
#include "gltf_reader.h"
#include "gltf_writer.h"
#include "../utility/VertexDuplicationUtilities.h"

namespace pmp_character
{
    /// \brief Generic function for loading  an attribute buffer with the type T
    /// \tparam T The type of the buffer to be read. Needs to be available at runtime
    /// \param primitive The mesh for which the buffer should be loaded
    /// \param model The model loaded by tinygltf
    /// \param attributeName The name of the attribute such as POSITION or NORMAL
    /// \param size out parameter for getting the size of the bfufer
    /// \return const pointer to buffer with type T
    template <typename T>
    const T *getTinyGLTFPrimitiveAttributeBuffer(tinygltf::Primitive primitive, tinygltf::Model &model,
                                                 const std::string &attributeName, int &size)
    {
        const tinygltf::Accessor &vertexAccessor = model.accessors[primitive.attributes[attributeName]];
        const tinygltf::BufferView &vertexBufferView = model.bufferViews[vertexAccessor.bufferView];
        const tinygltf::Buffer &vertexBuffer = model.buffers[vertexBufferView.buffer];
        const T *bufferContent =
            reinterpret_cast<const T *>(&vertexBuffer.data[vertexBufferView.byteOffset + vertexAccessor.byteOffset]);
        size = vertexAccessor.count;
        return bufferContent;
    }


    bool gltf_file::write(Character &character_, const std::filesystem::path& filename)
    {
        std::cout << "[INFO] Saving character to " << filename << std::endl;
        pmp::StopWatch timer;
        timer.start();
        tinygltf::TinyGLTF tinyGltf;
        tinygltf::Model model_;
        Character character = character_;
        // Make the overall structure
        // First create Joints
        for (auto joint : character.skeleton_.joints_)
        {
            tinygltf::Node node;
            node.name = joint->name_;
            // Take every entry of th e transform and put it into the node
            mat4ToVector(pmp::mat4(joint->local_), node.matrix);
            for (auto child : joint->children_)
            {
                node.children.emplace_back(child->id);
            }
            model_.nodes.emplace_back(node);
        }
        // Create the root joint separatly
        tinygltf::Node rootNode;
        auto rootJoint = character.skeleton_.root_;
        rootNode.name = rootJoint->name_;
        // Take every entry of th e transform and put it into the node
        mat4ToVector(pmp::mat4(rootJoint->local_), rootNode.matrix);
        for (auto child : rootJoint->children_)
        {
            rootNode.children.emplace_back(child->id);
        }
        model_.nodes.emplace_back(rootNode);
        // Create Morph Target Names
        // Add Extras
        std::vector<std::string> blendshapeNames;
        for (auto &blendshapeName : character.blendshapeNames)
        {
            // Put name into targets
            blendshapeNames.emplace_back(blendshapeName);
        }
        // Skin joint indices
        tinygltf::Skin skin;
        for (auto &joint : character.skeleton_.joints_)
        {
            skin.joints.emplace_back(joint->id);
        }
        model_.skins.emplace_back(skin);
        gltf_writer::createInverseBindMatrixBuffer(model_, character);
        for (size_t i=0; i< character.children.size(); i++)
        {

            auto child = *character.get_mesh_by_index(i);
            pmp::SurfaceMesh meshCopy;
            // std::cout << meshCopy.n_vertices() << std::endl;
            mesh_utilities::duplicateVerticesForExport(child, meshCopy);
            child.mesh = meshCopy;
            mesh_utilities::duplicateBlendshapeEntries(character,i);
            gltf_writer::createMesh(blendshapeNames, child, model_);
        }
        // Textures
        tinygltf::Texture texture;
        texture.name = "Default Texture";
        texture.source = 0;
        model_.textures.emplace_back(texture);
        // TODO Binary or as files?
        bool binary = true;
        if (filename.extension() == ".gltf")
        {
            binary = false;
        }
        bool success = false;
        if (binary)
        {
            gltf_writer::writeTexturesBinary(character, model_);
            success = tinyGltf.WriteGltfSceneToFile(&model_, filename.string(), true, true, false, false);
        }
        else
        {
            gltf_writer::writeImageSTBI(character, model_, filename);
            success = tinyGltf.WriteGltfSceneToFile(&model_, filename.string(), false, true, false, false);
        }
        // tinyGltf.SetPreserveImageChannels(true);
        if (not success)
        {
            std::cerr << "Could not write Character to " << filename << std::endl;
        }
        timer.stop();
        std::cout << "Saving finished. Took " << timer.elapsed() << "ms \n";
        return true;
    }


    bool gltf_file::load(const std::shared_ptr<Character> &characterPointer, const std::filesystem::path& filename)
    {
        pmp::StopWatch timer;
        timer.start();
        tinygltf::Model model;
        if (filename.extension() == ".glb")
        {
            loader.LoadBinaryFromFile(&model, &err, &warn, filename.string());
        }
        else if (filename.extension() == ".gltf")
        {
            // fprintf(stderr, "[DEBUG] Loading %s\n", filename.c_str());
            if (!loader.LoadASCIIFromFile(&model, &err, &warn, filename.string()))
            {
                fprintf(stderr, "[DEBUG] GLTF ERROR  \n%s\n", err.c_str());
                fprintf(stderr, "[DEBUG] GLTF WARNING\n%s\n", warn.c_str());
            }
        }
        else
        {
            std::cerr << "Wrong file format given. Please specify a .gltf or .glb file" << std::endl;
            exit(-1);
        }
        // std::cout << model.meshes.size() << std::endl;
        if (model.meshes.empty())
        {
            std::cerr << "Error: Empty Model loaded";
            return false;
        }
        bool addedRootNode = false;
        if (not model.skins.empty())
        {
            addedRootNode = gltf_reader::loadJoints(model, characterPointer);
        }
        gltf_reader::readBlendshapeTargetNames(characterPointer, model);

        for (auto &gltfNode : model.nodes)
        {
            if (gltfNode.mesh != -1)
            {
                tinygltf::Mesh &mesh = model.meshes[gltfNode.mesh];
                auto node = std::make_shared<MeshNode>();
                for (auto &primitive : mesh.primitives)
                {
                    node->name = mesh.name;
                    gltf_reader::readAllVertices(primitive, model, node->mesh);
                    gltf_reader::readAllNormals(primitive, model, node->mesh);
                    gltf_reader::readAllFaces(model, node, primitive);
                    gltf_reader::readTexcoords(node, primitive, model);
                    gltf_reader::readBlendshapes(model, primitive, characterPointer, node);
                    gltf_reader::readSkinningWeights(node, primitive, model, addedRootNode);
                }
                node->name = mesh.name;
                node->transform = getNodeTransform(gltfNode);
                pmp::SurfaceMesh uniqueVertexMesh;
                mesh_utilities::deleteDuplicateVertices(node->mesh, uniqueVertexMesh);
                // mesh_utilities::copyAllProperties(node.mesh, uniqueVertexMesh);
                mesh_utilities::deleteDuplicateVerticesInBlendshapes(
                    node->mesh, node->blendshape_collection);
                node->mesh = uniqueVertexMesh;
                characterPointer->children.emplace_back(node);
            }
        }
        for (auto &child : characterPointer->children)
        {
            auto childMeshNode = *std::dynamic_pointer_cast<MeshNode>(child);
            auto skinned_vertices = childMeshNode.mesh.add_vertex_property<pmp::vec3>(properties::vertexSkinnedPos);
            auto blend_position = childMeshNode.mesh.add_vertex_property<pmp::vec3>(properties::vertexBlendPosition);
            auto blend_normal = childMeshNode.mesh.add_vertex_property<pmp::vec3>(properties::vertexBlendNormal);
            auto skinned_normal = childMeshNode.mesh.add_vertex_property<pmp::vec3>(properties::vertexSkinnedNormal);
            auto global_pos = childMeshNode.mesh.add_vertex_property<pmp::vec3>(properties::vertexGlobalPosition);
            auto global_normal = childMeshNode.mesh.add_vertex_property<pmp::vec3>(properties::vertexGlobalNormal);
            auto normals = childMeshNode.mesh.get_vertex_property<pmp::vec3>(properties::vertexNormals);
            for (auto v : childMeshNode.mesh.vertices())
            {
                skinned_vertices[v] = childMeshNode.mesh.position(v);
                skinned_normal[v] = normals[v];
                global_pos[v] = childMeshNode.mesh.position(v);
                global_normal[v] = normals[v];
                blend_position[v] = childMeshNode.mesh.position(v);
                blend_normal[v] = normals[v];
            }
        }
        try
        {
            gltf_reader::readTexture(model, characterPointer, filename);
        }
        catch (pmp::IOException &e)
        {
            std::cerr << e.what() << std::endl;
        }
        characterPointer->calculate_size_and_center();
        characterPointer->skeleton_.root_->initialize_kinematics();

        // for (auto &node : character.children)
        // {
        //     std::cout << "Mesh " << node.name << " has " << node.mesh.n_vertices() << " Vertices\n";
        // }

        timer.stop();
        std::cout << "Loading finished. Took " << timer.elapsed() << "ms \n";
        return true;
    }


} // namespace pmp_character
