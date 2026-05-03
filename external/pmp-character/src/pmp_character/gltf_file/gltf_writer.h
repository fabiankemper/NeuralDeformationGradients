//
// Created by peasman on 04.01.24.
//

#ifndef GLTF_WRITER_H
#define GLTF_WRITER_H
#include "../PropertyNames.h"

#include "../node_types/Character.h"
#include "gltf_file.h"

namespace pmp_character::gltf_writer
{

    static tinygltf::Material &createMaterial(const MeshNode &child, tinygltf::Material &material)
    {
        material.name = child.name + "Material";
        tinygltf::Parameter baseColorTexture;
        // TODO Currently Hardcored Textureindex
        baseColorTexture.json_double_value["index"] = 0;
        tinygltf::Parameter roughnessFactor;
        roughnessFactor.number_value = 0.89;
        material.values["baseColorTexture"] = baseColorTexture;
        material.values["roughnessFactor"] = roughnessFactor;
        material.pbrMetallicRoughness.baseColorTexture.index = 0;
        material.pbrMetallicRoughness.baseColorTexture.texCoord = 0;
        return material;
    }

    static void createSkinningBuffers(const MeshNode &child, tinygltf::Model &model_, tinygltf::Primitive &primitive)
    {
        auto meshWeights = child.mesh.get_vertex_property<vec8>(properties::vertexSkinningWeights);
        auto jointDepends = child.mesh.get_vertex_property<ivec8>(properties::vertexSkinningDepends);
        std::vector<gltf_ivec4> joints;
        for (auto joint : jointDepends.vector())
        {
            joints.emplace_back(joint[0], joint[1], joint[2], joint[3]);
        }
        std::vector<pmp::vec4> weights;
        for (auto weight : meshWeights.vector())
        {
            weights.emplace_back(weight[0], weight[1], weight[2], weight[3]);
        }
        tinygltf::Buffer v_dependsBuffer;
        v_dependsBuffer.name = child.name + ":vDependsBuffer";
        tinygltf::Accessor v_dependsBufferAccessor;
        v_dependsBufferAccessor.name = child.name + ":vDependsBufferAccessor";
        tinygltf::BufferView v_dependsBufferView;
        v_dependsBufferView.target = TINYGLTF_TARGET_ARRAY_BUFFER;
        v_dependsBufferView.name = child.name + ":vDependsBufferView";
        gltf_file::createBuffer<gltf_ivec4>(model_, v_dependsBuffer, v_dependsBufferView, v_dependsBufferAccessor,
                                            joints);
        primitive.attributes["JOINTS_0"] = static_cast<int>(model_.accessors.size() - 1);
        tinygltf::Buffer v_weightsBuffer;
        v_weightsBuffer.name = child.name + ":v_weightsBuffer";
        tinygltf::Accessor v_weightsBufferAccessor;
        v_weightsBufferAccessor.name = child.name + ":v_weightsBufferAccessor";
        tinygltf::BufferView v_weightsBufferView;
        v_weightsBufferView.name = child.name + ":v_weightsBufferView";
        v_weightsBufferView.target = TINYGLTF_TARGET_ARRAY_BUFFER;
        gltf_file::createBuffer<pmp::vec4>(model_, v_weightsBuffer, v_weightsBufferView, v_weightsBufferAccessor, weights);
        primitive.attributes["WEIGHTS_0"] = static_cast<int>(model_.accessors.size() - 1);
    }

    static void createTexcoordsBuffer(const MeshNode &child, tinygltf::Model &model_, tinygltf::Primitive &primitive)
    {
        std::vector<pmp::TexCoord> &texcoords =
            child.mesh.get_vertex_property<pmp::TexCoord>(properties::vertexTextureCoordinates).vector();
        tinygltf::Buffer texcoordsBuffer;
        texcoordsBuffer.name = child.name + ":TexCoordsBuffer";
        tinygltf::Accessor texcoordsBufferAccessor;
        texcoordsBufferAccessor.name = child.name + ":TexCoordsBufferAccessor";
        tinygltf::BufferView texcoordsBufferView;
        texcoordsBufferView.target = TINYGLTF_TARGET_ARRAY_BUFFER;
        gltf_file::createBuffer<pmp::TexCoord>(model_, texcoordsBuffer, texcoordsBufferView, texcoordsBufferAccessor,
                                          texcoords);
        primitive.attributes["TEXCOORD_0"] = static_cast<int>(model_.accessors.size() - 1);
    }

    static void createNormalsBuffer(const MeshNode &child, tinygltf::Model &model_, tinygltf::Primitive &primitive)
    {
        std::vector<pmp::Normal> &normals = child.mesh.get_vertex_property<pmp::Normal>(properties::vertexNormals).vector();
#if PMP_SCALAR_TYPE_64
        std::vector<pmp::vec3> convertedNormals(normals.size());
        for(size_t i =0; i < normals.size(); i++)
        {
            convertedNormals[i] = pmp::vec3(normals[i]);
        }
#endif
        tinygltf::Buffer normalsBuffer;
        normalsBuffer.name = child.name + ":NormalsBuffer";
        tinygltf::Accessor normalsBufferAccessor;
        normalsBufferAccessor.name = child.name + ":NormalsBufferAccessor";
        tinygltf::BufferView normalsBufferView;
        normalsBufferView.target = TINYGLTF_TARGET_ARRAY_BUFFER;
#if PMP_SCALAR_TYPE_64
        gltf_file::createBuffer<pmp::vec3>(model_, normalsBuffer, normalsBufferView, normalsBufferAccessor, convertedNormals);
#else
        gltf_file::createBuffer<pmp::Point>(model_, normalsBuffer, normalsBufferView, normalsBufferAccessor, normals);
#endif
        primitive.attributes["NORMAL"] = static_cast<int>(model_.accessors.size() - 1);
    }

    static void createPositionsBuffer(MeshNode &child, tinygltf::Model &model_, tinygltf::Primitive &primitive)
    {
        std::vector<pmp::Point> &positions = child.mesh.positions();
#if PMP_SCALAR_TYPE_64
        std::vector<pmp::vec3> convertedPositions(positions.size());
        for(size_t i =0; i < positions.size(); i++)
        {
            convertedPositions[i] = pmp::vec3(positions[i]);
        }
#endif
        tinygltf::Buffer positionsBuffer;
        positionsBuffer.name = child.name + ":PositionBuffer";
        tinygltf::Accessor positionsBufferAccessor;
        positionsBufferAccessor.name = child.name + ":positionsBufferAccessor";
        tinygltf::BufferView positionsBufferView;
        positionsBufferView.name = child.name + ":PositionBufferView";
        positionsBufferView.target = TINYGLTF_TARGET_ARRAY_BUFFER;
#if PMP_SCALAR_TYPE_64
        gltf_file::createBuffer<pmp::vec3>(model_, positionsBuffer, positionsBufferView, positionsBufferAccessor, convertedPositions);
#else
        gltf_file::createBuffer<pmp::Point>(model_, positionsBuffer, positionsBufferView, positionsBufferAccessor, positions);
#endif
        primitive.attributes["POSITION"] = static_cast<int>(model_.accessors.size() - 1);
    }
    static void createMorphTargets(MeshNode &child, tinygltf::Model &model, tinygltf::Primitive &primitive)
    {
        // Get the fitting blendshape
        auto &blendshapes = child.blendshape_collection;

        for (auto &blendshape : blendshapes.blendshapes)
        {
            // Sparse or not?
            bool sparse = false;
            bool sparseNormal = false;
            if (blendshape.movingVertices.size() < child.mesh.n_vertices())
            {
                sparse = true;
            }
            if (blendshape.movingNormals.size() < child.mesh.n_vertices())
            {
                sparseNormal = true;
            }
            std::map<std::string, int> target;
            if (sparse)
            {
                std::vector<uint> movingVertices;
                std::vector<pmp::vec3> positions;
                for (size_t i = 0; i < blendshape.movingVertices.size(); i++)
                {
                    movingVertices.emplace_back(blendshape.movingVertices[i].idx());
                    positions.emplace_back(blendshape.deformation[i]);
                }
                gltf_file::createSparseTargetBuffer(model, movingVertices, positions, child.mesh.n_vertices());
                target["POSITION"] = model.accessors.size() - 1;
            }
            else
            {
                std::vector<pmp::vec3> denseValues(child.mesh.n_vertices(), pmp::vec3(0));
                for (size_t i = 0; i < blendshape.movingVertices.size(); i++)
                {
                    auto &index = blendshape.movingVertices[i];
                    denseValues[index.idx()] = pmp::vec3(blendshape.deformation[i]);
                }
                // Its dense we only need one buffer for all values
                tinygltf::Buffer positionBuffer;
                positionBuffer.name = "morphTargetsBuffer";
                tinygltf::BufferView positionBufferView;
                tinygltf::Accessor positionAccessor;
                positionAccessor.name = "blendshapePositionAccessor";
                positionBufferView.target = TINYGLTF_TARGET_ARRAY_BUFFER;
                gltf_file::createBuffer<pmp::vec3>(model, positionBuffer, positionBufferView, positionAccessor, denseValues);
                //                model.accessors.emplace_back(positionAccessor);
                target["POSITION"] = model.accessors.size() - 1;
            }
            if(sparseNormal)
            {
                std::vector<uint> movingNormals;
                std::vector<pmp::vec3> normals;
                for (size_t i = 0; i < blendshape.movingNormals.size(); i++)
                {
                    movingNormals.emplace_back(blendshape.movingNormals[i].idx());
                    normals.emplace_back(blendshape.normalDeformation[i]);
                }
                gltf_file::createSparseTargetBuffer(model, movingNormals, normals, child.mesh.n_vertices());
                target["NORMAL"] = model.accessors.size() -1;
            } else
            {
                std::vector<pmp::vec3> denseNormalValues(child.mesh.n_vertices(), pmp::vec3(0));
                for (size_t i = 0; i < blendshape.movingVertices.size(); i++)
                {
                    auto &index = blendshape.movingVertices[i];
                    denseNormalValues[index.idx()] = pmp::vec3(blendshape.normalDeformation[i]);
                }
                // Its dense we only need one buffer for all values
                tinygltf::Buffer normalBuffer;
                normalBuffer.name = "morphTargetsBuffer";
                tinygltf::BufferView normalBufferView;
                tinygltf::Accessor normalAccessor;
                normalAccessor.name = "blendshapeNormalAccessor";
                normalBufferView.target = TINYGLTF_TARGET_ARRAY_BUFFER;
                gltf_file::createBuffer<pmp::vec3>(model, normalBuffer, normalBufferView, normalAccessor, denseNormalValues);
                //                model.accessors.emplace_back(positionAccessor);
                target["NORMAL"] = model.accessors.size() -1 ;
            }
            primitive.targets.emplace_back(target);
        }


        // Create the targets in the primitive
    }
    static void createFaceBuffer(MeshNode &child, tinygltf::Model &model, tinygltf::Primitive &primitive)
    {
        std::vector<uint> faceIndices;
        for (auto f : child.mesh.faces())
        {
            for (auto v : child.mesh.vertices(f))
            {
                faceIndices.emplace_back(v.idx());
            }
        }
        tinygltf::Buffer indexBuffer;
        indexBuffer.name = child.name + ":FaceIndexBuffer";
        tinygltf::Accessor indexAccessor;
        indexAccessor.name = child.name + ":FaceIndexBufferAccessor";
        tinygltf::BufferView indexBufferView;
        indexBufferView.name = child.name + ":FaceIndexBufferView";
        gltf_file::createBuffer<uint>(model, indexBuffer, indexBufferView, indexAccessor, faceIndices);
        primitive.indices = static_cast<int>(model.accessors.size() - 1);
        //        indexBufferView.target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;
        model.bufferViews[model.bufferViews.size() - 1].target = TINYGLTF_TARGET_ELEMENT_ARRAY_BUFFER;
        model.accessors[model.accessors.size() - 1].minValues = {};
        model.accessors[model.accessors.size() - 1].maxValues = {};
    }

    static void createInverseBindMatrixBuffer(tinygltf::Model &model, Character &character)
    {
        std::vector<pmp::mat4> inverseBindMatrices;
        for (auto &joint : character.skeleton_.joints_)
        {
            inverseBindMatrices.emplace_back(inverse(joint->global_initial_));
        }
        tinygltf::Buffer buffer;
        tinygltf::BufferView bufferView;
        tinygltf::Accessor accessor;
        bufferView.name = "InverseMatrixBufferView";
        accessor.name = "InverseMatrixBufferAccessor";
        gltf_file::createBuffer<pmp::mat4>(model, buffer, bufferView, accessor, inverseBindMatrices);
        bufferView.target = TINYGLTF_TARGET_ARRAY_BUFFER;
        model.skins[0].inverseBindMatrices = model.accessors.size() - 1;
    }
    static void createMesh(std::vector<std::string> &blendshapeNames, MeshNode &child,
                           tinygltf::Model &model_)
    {
        // Create a new node
        tinygltf::Node node;
        // Create a new mesh
        tinygltf::Mesh mesh;
        // Create new Primitive
        tinygltf::Primitive primitive;
        primitive.mode = 4;
        // First get all Positions
        createPositionsBuffer(child, model_, primitive);
        // Get all Normals
        createNormalsBuffer(child, model_, primitive);
        // Get all Faces
        createFaceBuffer(child, model_, primitive);
        // TexCoords
        createTexcoordsBuffer(child, model_, primitive);

        // Joints and weights
        createSkinningBuffers(child, model_, primitive);

        // Morph Targets
        createMorphTargets(child, model_, primitive);
        mesh.name = child.name;
        //            mesh.extras.Keys().emplace_back("targetNames");
        std::vector<tinygltf::Value> values;
        for (const std::string &name : blendshapeNames)
        {
            values.emplace_back(name);
        }
        tinygltf::Value targetNamesArray(values);
        std::map<std::string, tinygltf::Value> objectMap;
        objectMap["targetNames"] = targetNamesArray;
        tinygltf::Value::Object object(objectMap);
        tinygltf::Value valueObject(object);
        mesh.extras = valueObject;
        node.mesh = model_.meshes.size();
        node.name = child.name;
        node.skin = model_.skins.size() - 1;
        // Create Material
        tinygltf::Material material;
        material = createMaterial(child, material);

        primitive.material = model_.materials.size();
        node.skin = 0;
        model_.materials.emplace_back(material);
        model_.nodes.emplace_back(node);
        mesh.primitives.emplace_back(primitive);
        model_.meshes.emplace_back(mesh);
    }
    static void writeTexturesBinary(Character &character, tinygltf::Model &model_)
    {
        for (auto& texturePair : character.textureAtlas)
        {
            auto &currentTexture = texturePair.second;
            tinygltf::Image image;
            image.width = currentTexture.width;
            image.height = currentTexture.height;
            image.name = currentTexture.name;
            image.component = currentTexture.numComponents;
            size_t byteSize = currentTexture.width * currentTexture.height * currentTexture.numComponents;
            image.bits = 8;
            auto imageVector =
                std::vector<stbi_uc>(currentTexture.textureBuffer, currentTexture.textureBuffer + (byteSize));
            image.pixel_type = TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE;
            image.image = imageVector;

            tinygltf::Buffer imageBuffer;
            imageBuffer.data = imageVector;
            imageBuffer.name = image.name + "Buffer";
            imageBuffer.uri = "";
            // imageBuffer.
            tinygltf::BufferView imageBufferView;
            imageBufferView.buffer = model_.buffers.size();
            // model_.buffers.emplace_back(imageBuffer);
            imageBufferView.byteLength = byteSize;
            imageBufferView.byteOffset = 0;
            imageBufferView.name = image.name + "BufferView";

            // Create a string from the char array
            image.mimeType = "image/png";

            model_.images.emplace_back(image);
        }
    }
    static void writeImageSTBI(Character &character, tinygltf::Model &model_, std::filesystem::path target_folder)
    {
        for (auto &[textture_name, texture] : character.textureAtlas)
        {
            auto &currentTexture = texture;
            tinygltf::Image image;
            image.width = currentTexture.width;
            image.height = currentTexture.height;
            image.name = currentTexture.name;
            image.component = 4;
            image.pixel_type = TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE;
            if(is_regular_file(target_folder) || not std::filesystem::exists(target_folder))
            {
                target_folder = target_folder.remove_filename();
            }
            //            size_t size = image.width * image.height;
            if (currentTexture.filepath.empty())
            {
                image.uri = currentTexture.name + ".png";
            }
            else
            {
                image.uri = currentTexture.filepath.string();
            }
            auto outputPath = target_folder/image.uri;

            image.bits = 8;
            model_.images.emplace_back(image);
            std::cout << "[INFO] Writing " << image.name << " to " << outputPath << std::endl;
            stbi_write_png(outputPath.string().c_str(), currentTexture.width, currentTexture.height, STBI_rgb_alpha,
                           currentTexture.textureBuffer, 0);
        }
    }



} // namespace pmp_character::gltf_writer


#endif // GLTF_WRITER_H
