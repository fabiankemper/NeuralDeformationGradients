//
// Created by peasman on 03.01.24.
//

#ifndef GLTF_READER_H
#define GLTF_READER_H
#include <pmp/mat_vec.h>
#include <pmp/surface_mesh.h>

#include "../node_types/Character.h"

namespace pmp_character::gltf_reader
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
                                                 std::string attributeName, int &size)
    {
        const tinygltf::Accessor &vertexAccessor = model.accessors[primitive.attributes[attributeName]];
        const tinygltf::BufferView &vertexBufferView = model.bufferViews[vertexAccessor.bufferView];
        const tinygltf::Buffer &vertexBuffer = model.buffers[vertexBufferView.buffer];
        const T *bufferContent =
            reinterpret_cast<const T *>(&vertexBuffer.data[vertexBufferView.byteOffset + vertexAccessor.byteOffset]);
        size = vertexAccessor.count;
        return bufferContent;
    }

    static void readAllVertices(tinygltf::Primitive primitive, tinygltf::Model &model, pmp::SurfaceMesh &mesh)
    {
        int size = 0;
        const float *positions = getTinyGLTFPrimitiveAttributeBuffer<float>(primitive, model, "POSITION", size);
        for (int i = 0; i < size; ++i)
        {
            mesh.add_vertex(pmp::Point(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]));
        }
    }

    static void readAllNormals(tinygltf::Primitive primitive, tinygltf::Model &model, pmp::SurfaceMesh &mesh)
    {
        int size = 0;
        auto normals = mesh.get_vertex_property<pmp::Normal>(properties::vertexNormals);
        const float *positions = getTinyGLTFPrimitiveAttributeBuffer<float>(primitive, model, "NORMAL", size);
        for (int i = 0; i < size; ++i)
        {
            // Normals are Vec3 components, so for each vec3 stride, offset for x, y, and z.
            normals[pmp::Vertex(i)] = pmp::Normal(positions[i * 3 + 0], positions[i * 3 + 1], positions[i * 3 + 2]);
        }
    }

    static void readAllFaces(tinygltf::Model &model, std::shared_ptr<MeshNode> node, tinygltf::Primitive &primitive)
    {
        if (primitive.indices >= 0)
        {
            const tinygltf::Accessor &accessor = model.accessors[primitive.indices];
            const tinygltf::BufferView &bufferView = model.bufferViews[accessor.bufferView];
            const tinygltf::Buffer &buffer = model.buffers[bufferView.buffer];
            // Assuming the indices are 16-bit unsigned integers (GL_UNSIGNED_SHORT)
            // Get type of buffer
            if (accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT)
            {
                const uint16_t *indices =
                    reinterpret_cast<const uint16_t *>(&buffer.data[bufferView.byteOffset + accessor.byteOffset]);
                // Now 'indices' contains the faces of the mesh
                // You can iterate over them and process accordingly
                // Currently only supports triangle meshes
                // printf("Found %lu indices in mesh \n", accessor.count);
                for (size_t i = 0; i < accessor.count; i += 3)
                {
                    std::vector<pmp::Vertex> face_vertices;
                    face_vertices.emplace_back(indices[i]);
                    face_vertices.emplace_back(indices[i + 1]);
                    face_vertices.emplace_back(indices[i + 2]);
                    try
                    {
                        node->mesh.add_face(face_vertices);
                    }
                    catch (pmp::TopologyException &e)
                    {
                        std::cout << e.what() << std::endl;
                    }
                }
            }
            else if (accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT)
            {
                const uint32_t *indices =
                    reinterpret_cast<const uint32_t *>(&buffer.data[bufferView.byteOffset + accessor.byteOffset]);
                // Now 'indices' contains the faces of the mesh
                // You can iterate over them and process accordingly
                // Currently only supports triangle meshes
                // printf("Found %lu indices in mesh \n", accessor.count);
                for (size_t i = 0; i < accessor.count; i += 3)
                {
                    std::vector<pmp::Vertex> face_vertices;
                    face_vertices.emplace_back(indices[i]);
                    face_vertices.emplace_back(indices[i + 1]);
                    face_vertices.emplace_back(indices[i + 2]);
                    try
                    {
                        node->mesh.add_face(face_vertices);
                    }
                    catch (pmp::TopologyException &e)
                    {
                        std::cout << e.what() << std::endl;
                    }
                }
            }
        }
    }

    static void readBlendshapeTargetNames(const std::shared_ptr<Character> &character, tinygltf::Model &model)
    {
        // Get all Target names from extras
        if (model.meshes[0].extras.Size() == 0)
            return;
        auto targets = model.meshes[0].extras.Get("targetNames");
        std::vector<std::string> blendshapeNames;
        if (targets.IsArray())
        {
            for (size_t i = 0; i < targets.ArrayLen(); i++)
            {
                // std::cout << (targets.Get(i).Get<std::string>()) << std::endl;
                blendshapeNames.emplace_back(targets.Get(i).Get<std::string>());
                character->blendshapeNames.emplace_back(targets.Get(i).Get<std::string>());
                character->blendshapeEffectors.emplace_back(0);
            }
        }
    }


    static void readInverseBindMatrices(const tinygltf::Model &model, const std::shared_ptr<Character> &character,
                                        const tinygltf::Skin &skin)
    {
        tinygltf::Accessor inverseBindAccessor = model.accessors[skin.inverseBindMatrices];
        tinygltf::BufferView inverseBindBufferView = model.bufferViews[inverseBindAccessor.bufferView];
        tinygltf::Buffer inverseBindBuffer = model.buffers[inverseBindBufferView.buffer];
        const pmp::mat4 *inverseBindMatrices = reinterpret_cast<const pmp::mat4 *>(
            &inverseBindBuffer.data[inverseBindBufferView.byteOffset + inverseBindAccessor.byteOffset]);
        for (size_t i = 0; i < inverseBindAccessor.count; i++)
        {
            character->skeleton_.joints_[i]->inverse_bind_matrix_ = inverseBindMatrices[i];
        }
    }

    static void createJointHierarchy(const tinygltf::Model &model, const std::shared_ptr<Character> &character, tinygltf::Skin &skin)
    {
        for (auto &jointIndex : skin.joints)
        {
            auto &node = model.nodes[jointIndex];
            auto parentJoint = character->skeleton_.get_joint(node.name);
            if (not node.children.empty())
            {
                for (auto &childIndex : node.children)
                {
                    auto &child = model.nodes[childIndex];
                    auto namedJoint = character->skeleton_.get_joint(child.name);
                    namedJoint->parent_ = parentJoint;
                    parentJoint->children_.emplace_back(namedJoint);
                }
            }
        }
    }

    /// Create the root joint if it is missing. If it was missing returns true. Else returns false
    /// @param model
    /// @param character
    /// @return
    static bool createRootJoint(tinygltf::Model &model, const std::shared_ptr<Character> &character)
    {
        bool needs_additional_root_joint = true;

        for (auto* joint : character->skeleton_.joints_)
        {
            if (joint->name_ == "root")
            {
                needs_additional_root_joint = false;
                character->skeleton_.root_ = joint;
                break;
            }
        }

        // Skeleton already has a "root" joint
        if (!needs_additional_root_joint)
        {
            return false;
        }

        // There was no joint named root.
        // Use a GLTF Node with name "root" or select a joint
        // with no parents as root joint.
        for (auto &node : model.nodes)
        {
            if (node.name == "root")
            {
                Joint *joint = new Joint("root", gltf_file::getNodeTransform(node));
                character->skeleton_.root_ = joint;
                character->skeleton_.joints_.insert(character->skeleton_.joints_.begin(), joint);
                break;
            }
        }
        // Fix hierarchy
        for (auto joint : character->skeleton_.joints_)
        {
            if (joint->parent_ == nullptr and not(joint->name_ == "root"))
            {
                joint->parent_ = character->skeleton_.root_;
                character->skeleton_.root_->children_.emplace_back(joint);
            }
        }
        return true;
    }

    static void readJoints(const tinygltf::Model &model, const std::shared_ptr<Character> &character, tinygltf::Skin &skin)
    {
        for (auto &jointIndex : skin.joints)
        {
            // get the node
            auto &node = model.nodes[jointIndex];
            // create the joint and put it in the right place inside the skeleton
            pmp::mat4 local = gltf_file::getNodeTransform(node);
            Joint *joint = new Joint(node.name, local);
            joint->id = character->skeleton_.joints_.size();
            character->skeleton_.joints_.emplace_back(joint);
        }
    }

    static void readTexcoords(std::shared_ptr<MeshNode> node, tinygltf::Primitive &primitive, tinygltf::Model &model)
    {
        const tinygltf::Accessor &vertexAccessor = model.accessors[primitive.attributes["TEXCOORD_0"]];
        const tinygltf::BufferView &vertexBufferView = model.bufferViews[vertexAccessor.bufferView];
        const tinygltf::Buffer &vertexBuffer = model.buffers[vertexBufferView.buffer];
        const pmp::TexCoord *uvs = reinterpret_cast<const pmp::TexCoord *>(
            &vertexBuffer.data[vertexBufferView.byteOffset + vertexAccessor.byteOffset]);
        auto texcoords = node->mesh.get_vertex_property<pmp::TexCoord>(properties::vertexTextureCoordinates);
        for (size_t i = 0; i < vertexAccessor.count; ++i)
        {
            //            TexCoord coord(uvs[i*2 + 0], uvs[i*2 +1]);
            texcoords[pmp::Vertex(i)] = uvs[i];
        }
    }

    static void readTexture(tinygltf::Model &model, const std::shared_ptr<Character>& character, std::filesystem::path filePath)
    {
        if (model.images.empty())
        {
            std::cerr << "No texture image found" << std::endl;
            return;
        }
        tinygltf::Image image = model.images[0];
        // std::cout << "Length: " << image.uri.length() << std::endl;
        if (not image.uri.empty())
        {
            // Image URI is not empy -> Load the image via stbi
            std::filesystem::path imageFilePath = filePath;
            stbi_set_flip_vertically_on_load(false);
            imageFilePath.replace_filename(model.images[0].uri);
            // texture_id = character.load_textures(imageFilePath);
            if (character->textureAtlas.find(image.name) == character->textureAtlas.end())
            {
                int width, height, num_channels;
                stbi_set_flip_vertically_on_load(false);
                unsigned char *img = stbi_load(imageFilePath.string().c_str(), &width, &height, &num_channels, STBI_rgb_alpha);
                if (!img)
                    throw pmp::IOException("Failed to load texture file: " + std::string(image.uri));
                // printf("Loaded Texture with size %lu, width: %d, height %d and %d channels", image.image.size(), width,
                //        height, num_channels);

                size_t imageByteSize = height * width * num_channels * sizeof(stbi_uc);
                Texture texture = {img, image.name, image.uri, imageByteSize, width, height, num_channels};
                character->textureAtlas[texture.name] = texture;
            }
        }
        else
        {

            // Image is embedded in the binary file. Load it from memory
            const tinygltf::BufferView &bufferView = model.bufferViews[image.bufferView];
            const tinygltf::Buffer &buffer = model.buffers[bufferView.buffer];
            const size_t offset = bufferView.byteOffset;
            const size_t size = bufferView.byteLength;

            const unsigned char *imageData = buffer.data.data() + offset;
            int width, height, numChannels;
            stbi_set_flip_vertically_on_load(false);
            unsigned char *decodedData =
                stbi_load_from_memory(imageData, size, &width, &height, &numChannels, STBI_rgb_alpha);
            // printf("Loaded Texture with size %lu, width: %d, height %d and %d channels", image.image.size(), width,
            //        height, numChannels);
            // texture_id = character.load_texture_from_memory(decodedData, width, height);
            Texture texture = {decodedData, image.name, image.uri, bufferView.byteLength,
                               width, height, numChannels};
            character->textureAtlas[texture.name] = texture;
        }
        for (auto &child : character->children)
        {
            auto childMeshNode = dynamic_pointer_cast<MeshNode>(child);
            childMeshNode->texture_name = image.name;
            childMeshNode->parent = character.get();
        }
    }

    /// \brief Read als skinning weights and joint depends out of the gltf file for a single mesh
    /// \param node The mesh node in which the surface mesh is
    /// \param primitive The mesh with the depends and weights
    /// \param addedRootNode
    static void readSkinningWeights(std::shared_ptr<MeshNode>& node, const tinygltf::Primitive &primitive, tinygltf::Model &model, bool addedRootNode = false)
    {

        int weights_size;
        const pmp::vec4 *weights = getTinyGLTFPrimitiveAttributeBuffer<pmp::vec4>(primitive, model, "WEIGHTS_0", weights_size);
        int joints_size;
        const gltf_ivec4 *joints =
            getTinyGLTFPrimitiveAttributeBuffer<gltf_ivec4>(primitive, model, "JOINTS_0", joints_size);
        // Get vertex Attributes
        auto meshWeights = node->mesh.get_vertex_property<vec8>(properties::vertexSkinningWeights);
        auto jointDepends = node->mesh.get_vertex_property<ivec8>(properties::vertexSkinningDepends);
        for (int i = 0; i < weights_size; i++)
        {
            pmp::Vertex currentVertex(i);
            ivec8 joint(0);
            if(addedRootNode)
            {
                joint[0] = joints[i][0] + 1;
                joint[1] = joints[i][1] + 1;
                joint[2] = joints[i][2] + 1;
                joint[3] = joints[i][3] + 1;
            } else
            {
                joint[0] = joints[i][0];
                joint[1] = joints[i][1];
                joint[2] = joints[i][2];
                joint[3] = joints[i][3];
            }

            //            joint = joints[i];

            vec8 weight(0);
            weight[0] = weights[i][0];
            weight[1] = weights[i][1];
            weight[2] = weights[i][2];
            weight[3] = weights[i][3];
            meshWeights[currentVertex] = weight;
            jointDepends[currentVertex] = joint;

        }
    }

    static void readBlendshapeNormalsSparse(tinygltf::Model &model, Blendshape &blendshape,
                                            const tinygltf::Accessor &deformationAccessor)
    {
        // Sparse so only some vertices got a deformation
        const tinygltf::BufferView &indexBufferView =
            model.bufferViews[deformationAccessor.sparse.indices.bufferView];
        const tinygltf::BufferView &indexValueView =
            model.bufferViews[deformationAccessor.sparse.values.bufferView];
        const tinygltf::Buffer &indexBuffer = model.buffers[indexBufferView.buffer];
        const tinygltf::Buffer &valueBuffer = model.buffers[indexValueView.buffer];
        const float *deformations = reinterpret_cast<const float *>(
            &valueBuffer.data[indexValueView.byteOffset + deformationAccessor.byteOffset]);

        // Blender exports some indices with no type (componenttype =0)
        if (deformationAccessor.sparse.indices.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT or
            deformationAccessor.sparse.indices.componentType == 0)
        {
            const uint16_t *indices = reinterpret_cast<const uint16_t *>(
                &indexBuffer.data[indexBufferView.byteOffset + deformationAccessor.byteOffset]);
            // Get every sparse index
            for (int i = 0; i < deformationAccessor.sparse.count; i++)
            {
                blendshape.movingNormals.emplace_back(indices[i]);
                blendshape.normalDeformation.emplace_back(deformations[i * 3 + 0], deformations[i * 3 + 1],
                                                          deformations[i * 3 + 2]);
            }
        }
        else
        {
            // If the indices are stored in unsigned int32
            const uint32_t *indices = reinterpret_cast<const uint32_t *>(
                &indexBuffer.data[indexBufferView.byteOffset + deformationAccessor.byteOffset]);
            // Get every sparse index
            for (int i = 0; i < deformationAccessor.sparse.count; i++)
            {
                blendshape.movingNormals.emplace_back(indices[i]);
                blendshape.normalDeformation.emplace_back(deformations[i * 3 + 0], deformations[i * 3 + 1],
                                                          deformations[i * 3 + 2]);
            }
        }
    }

    static void readBlendshapeSparse(tinygltf::Model &model,  Blendshape &blendshape,
                                     const tinygltf::Accessor &deformationAccessor)
    {
        // Sparse so only some vertices got a deformation
        const tinygltf::BufferView &indexBufferView =
            model.bufferViews[deformationAccessor.sparse.indices.bufferView];
        const tinygltf::BufferView &indexValueView =
            model.bufferViews[deformationAccessor.sparse.values.bufferView];
        const tinygltf::Buffer &indexBuffer = model.buffers[indexBufferView.buffer];
        const tinygltf::Buffer &valueBuffer = model.buffers[indexValueView.buffer];
        const float *deformations = reinterpret_cast<const float *>(
            &valueBuffer.data[indexValueView.byteOffset + deformationAccessor.byteOffset]);

        // Blender exports some indices with no type (componenttype =0)
        if (deformationAccessor.sparse.indices.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT or
            deformationAccessor.sparse.indices.componentType == 0)
        {
            const uint16_t *indices = reinterpret_cast<const uint16_t *>(
                &indexBuffer.data[indexBufferView.byteOffset + deformationAccessor.byteOffset]);
            // Get every sparse index
            for (int i = 0; i < deformationAccessor.sparse.count; i++)
            {
                blendshape.movingVertices.emplace_back(indices[i]);
                blendshape.deformation.emplace_back(deformations[i * 3 + 0], deformations[i * 3 + 1],
                                                    deformations[i * 3 + 2]);
            }
        }
        else
        {
            // If the indices are stored in unsigned int32
            const uint32_t *indices = reinterpret_cast<const uint32_t *>(
                &indexBuffer.data[indexBufferView.byteOffset + deformationAccessor.byteOffset]);
            // Get every sparse index
            for (int i = 0; i < deformationAccessor.sparse.count; i++)
            {
                blendshape.movingVertices.emplace_back(indices[i]);
                blendshape.deformation.emplace_back(deformations[i * 3 + 0], deformations[i * 3 + 1],
                                                    deformations[i * 3 + 2]);
            }
        }

    }

    static void readBlendshapeNormalsDense(tinygltf::Model &model,  Blendshape &blendshape,
                                           const tinygltf::Accessor &deformationAccessor)
    {
        const tinygltf::BufferView &normalDeformationBufferView = model.bufferViews[deformationAccessor.bufferView];
        const tinygltf::Buffer &normalDeformationBuffer = model.buffers[normalDeformationBufferView.buffer];
        const float *normalDeformations = reinterpret_cast<const float *>(
            &normalDeformationBuffer.data[normalDeformationBufferView.byteOffset + deformationAccessor.byteOffset]);
        for (size_t i = 0; i < deformationAccessor.count; i++)
        {
            blendshape.movingNormals.emplace_back(i);
            blendshape.normalDeformation.emplace_back(normalDeformations[i * 3 + 0], normalDeformations[i * 3 + 1],
                                                      normalDeformations[i * 3 + 2]);
        }
    }

    static void readBlendshapeDense(tinygltf::Model &model,Blendshape &blendshape,
                                    const tinygltf::Accessor &deformationAccessor)
    {
        // Not sparse so every Vertex has a deformation
        const tinygltf::BufferView &deformationBufferView = model.bufferViews[deformationAccessor.bufferView];
        const tinygltf::Buffer &deformationBuffer = model.buffers[deformationBufferView.buffer];
        const float *deformations = reinterpret_cast<const float *>(
            &deformationBuffer.data[deformationBufferView.byteOffset + deformationAccessor.byteOffset]);

        for (size_t i = 0; i < deformationAccessor.count; i++)
        {
            blendshape.movingVertices.emplace_back(i);
            blendshape.deformation.emplace_back(deformations[i * 3 + 0], deformations[i * 3 + 1],
                                                deformations[i * 3 + 2]);
        }
    }

    static void readBlendshapes(tinygltf::Model &model, tinygltf::Primitive primitive, const std::shared_ptr<Character> &character, std::shared_ptr<
                                MeshNode>& node)
    {
        if (primitive.targets.empty())
        {
            std::cerr << "Empty Targets" << std::endl;
            BlendshapeCollection blendshape_collection;
            blendshape_collection.empty = true;
            node->blendshape_collection = blendshape_collection;
            return;
        }
        // printf("Number of targets is: %lu \n", primitive.targets.size());
        int j = 0;
        BlendshapeCollection blendshapes;
        for (auto target : primitive.targets)
        {
            Blendshape blendshape(character->blendshapeNames[j]);
            const tinygltf::Accessor &deformationAccessor = model.accessors[target["POSITION"]];
            const tinygltf::Accessor &deformationNormalAccessor = model.accessors[target["NORMAL"]];
            if (deformationAccessor.sparse.isSparse)
            {
                readBlendshapeSparse(model,  blendshape, deformationAccessor);
            }
            else
            {
                readBlendshapeDense(model, blendshape, deformationAccessor);
            }
            if (deformationNormalAccessor.sparse.isSparse)
            {
                readBlendshapeNormalsSparse(model, blendshape, deformationNormalAccessor);
            }
            else
            {
                readBlendshapeNormalsDense(model, blendshape, deformationNormalAccessor);
            }
            j++;
            blendshapes.blendshapes.emplace_back(blendshape);
        }
        node->blendshape_collection = blendshapes;
    }

    static bool loadJoints(tinygltf::Model &model, const std::shared_ptr<Character> &character)
    {
        std::map<std::string, std::vector<int>> jointToChildrenMap;
        std::vector<std::tuple<Joint *, int>> jointsUnsorted;
        auto &skin = model.skins[0];
        // Read Joints in right order
        readJoints(model, character, skin);
        // Set the joints into the Character
        // Read inverse Bind Matrices
        readInverseBindMatrices(model, character, skin);
        // Create the joint hierarchy
        createJointHierarchy(model, character, skin);
        return createRootJoint(model, character);
        // std::cout << "Done loading joints" << std::endl;
    }
} // namespace pmp_character::gltf_reader

#endif // GLTF_READER_H
