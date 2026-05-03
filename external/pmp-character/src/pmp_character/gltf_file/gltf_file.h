//
// Created by peasman on 07.12.23.
//

#ifndef GLTF_FILE_H
#define GLTF_FILE_H
#include <filesystem>
#include "tiny_gltf.h"

#include "../node_types/Character.h"

namespace pmp_character
{
    using gltf_ivec4 = pmp::Vector<int8_t, 4>;
    using uint = unsigned int;

    class gltf_file
    {

    public:
        // Vertex attributes
        // tinygltf::Model model;
        tinygltf::TinyGLTF loader;
        std::string err;
        std::string warn;

        bool load(const std::shared_ptr<Character> &characterPointer, const std::filesystem::path& filename);

        static bool write(Character &character, const std::filesystem::path& filename);

        static void createSparseTargetBuffer(tinygltf::Model &model, std::vector<uint> &indices,
                                             std::vector<pmp::vec3> &data, size_t maxNumber)
        {
            // Create Index Buffer
            tinygltf::Accessor accessor;
            tinygltf::Buffer indexBuffer;
            tinygltf::BufferView indexBufferView;
            size_t indexSize = data.size();
            size_t indexByteSize = indexSize * sizeof(uint);
            // Index Buffer View initialization
            indexBufferView.byteLength = indexByteSize;
            indexBufferView.byteOffset = 0;
            indexBufferView.buffer = model.buffers.size();
            indexBufferView.byteStride = 0; // sizeof(uint) ;
            indexBufferView.name = "SparseIndexBufferView";
            indexBufferView.target = 0;
            model.bufferViews.emplace_back(indexBufferView);

            // Index Buffer creation
            indexBuffer.data.resize(indexByteSize);
            indexBuffer.name = "SparseIndexBuffer";
            memcpy(indexBuffer.data.data(), indices.data(), indexByteSize);
            model.buffers.emplace_back(indexBuffer);

            // Create Position Buffer
            tinygltf::Buffer positionBuffer;
            positionBuffer.name = "positionBuffer";
            tinygltf::BufferView positionBufferView;
            size_t dataSize = data.size();
            size_t dataByteSize = dataSize * sizeof(pmp::vec3);

            // Position Buffer Creation
            positionBufferView.byteLength = dataByteSize;
            positionBufferView.byteOffset = 0;
            positionBufferView.buffer = model.buffers.size();
            positionBufferView.byteStride = 0; // sizeof(T);
            positionBufferView.name = "PositionBufferView";
            positionBufferView.target = 0;
            model.bufferViews.emplace_back(positionBufferView);
            positionBuffer.data.resize(dataByteSize);
            memcpy(positionBuffer.data.data(), data.data(), dataByteSize);
            model.buffers.emplace_back(positionBuffer);

            // Sparse Accessor creation
            accessor.sparse.isSparse = true;
            accessor.type = TINYGLTF_TYPE_VEC3;
            accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
            accessor.sparse.indices.bufferView = model.bufferViews.size() - 2;
            accessor.sparse.indices.byteOffset = 0;
            accessor.sparse.indices.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
            accessor.sparse.count = indexSize;
            accessor.count = indexSize;
            accessor.bufferView = -1;
            accessor.count = maxNumber;
            accessor.byteOffset = 0;
            accessor.sparse.values.bufferView = model.bufferViews.size() - 1;
            accessor.sparse.values.byteOffset = 0;
            gltf_file::setMinMaxValues(accessor, data);
            accessor.name = "sparseMorphTargetAccessor";
            model.accessors.emplace_back(accessor);
        }

        template <typename T>
        static void createBuffer(tinygltf::Model &model, tinygltf::Buffer &buffer, tinygltf::BufferView &bufferView,
                                 tinygltf::Accessor &accessor, std::vector<T> &data)
        {
            // How big is the current buffer list
            size_t size = data.size();
            size_t byteSize = size * sizeof(T);
            bufferView.byteLength = byteSize;
            bufferView.byteOffset = 0;
            bufferView.buffer = model.buffers.size();
            model.bufferViews.emplace_back(bufferView);
            // Target should be irrelevant
            // Add Buffer number to accessor
            gltf_file::createBufferAccessor<T>(accessor, data);
            accessor.bufferView = model.bufferViews.size() - 1;
            bufferView.byteStride = 0; // accessor.ByteStride(bufferView);
            accessor.byteOffset = 0;
            accessor.sparse.isSparse = false;
            model.accessors.emplace_back(accessor);
            // Create the buffer itself
            buffer.data.resize(byteSize);
            memcpy(buffer.data.data(), data.data(), byteSize);
            model.buffers.emplace_back(buffer);
        }
        template <typename T>
        static void createBufferAccessor(tinygltf::Accessor &accessor, std::vector<T> &data);

        static pmp::mat4 fromVector(std::vector<double> entries)
        {

            pmp::mat4 mat(0);
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    mat(j, i) = entries[i * 4 + j];
                }
            }
            return mat;
        }

        static pmp::mat4 getNodeTransform(const tinygltf::Node &node)
        {
            if (!node.matrix.empty())
            {
                return fromVector(node.matrix);
            }
            pmp::mat4 translation = pmp::mat4::identity();
            if (!node.translation.empty())
            {
                translation = translation_matrix(pmp::vec3(node.translation[0], node.translation[1], node.translation[2]));
            }
            pmp::mat4 rotation = pmp::mat4::identity();
            if (!node.rotation.empty())
                rotation =
                    pmp::rotation_matrix(pmp::vec4(node.rotation[0], node.rotation[1], node.rotation[2], node.rotation[3]));
            pmp::mat4 scale = pmp::mat4::identity();
            if (!node.scale.empty())
                scale = scaling_matrix(pmp::vec3(node.scale[0], node.scale[1], node.scale[2]));

            pmp::mat4 local = translation * rotation * scale;
            return local;
        }
        template <typename T>
        static T getMinValue(const std::vector<T> &values)
        {
            T result(std::numeric_limits<typename T::value_type>::max());

            for (const T& value : values)
            {
                for (size_t i = 0; i < T::rows(); ++i)
                {
                    result[i] = std::min(result[i], value[i]);
                }
            }

            return result;
        }

        template <typename T>
        static T getMaxValue(const std::vector<T> &values)
        {
            T result(std::numeric_limits<typename T::value_type>::lowest());

            for (const T& value : values)
            {
                for (size_t i = 0; i < T::rows(); ++i)
                {
                    result[i] = std::max(result[i], value[i]);
                }
            }

            return result;
        }

        static void mat4ToVector(pmp::mat4 mat, std::vector<double> &targetMatrix)
        {
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    targetMatrix.emplace_back(mat(j, i));
                }
            }
        }
        static std::vector<double> unrollVector(pmp::vec3 data) { return std::vector<double>{data[0], data[1], data[2]}; }

        static tinygltf::Material &createMaterial(const MeshNode &child, tinygltf::Material &material);

        static void createMesh(std::vector<std::string> &blendshapeNames, MeshNode &child, Character &character,
                               tinygltf::Model &model_);

        template <typename T>
        static void setMinMaxValues(tinygltf::Accessor &accessor, std::vector<T> &data)
        {
            auto min = getMinValue<T>(data);
            auto max = getMaxValue<T>(data);

            for (int i = 0; i < T::size(); ++i)
            {
                accessor.minValues.emplace_back(min[i]);
            }
            for (int i = 0; i < T::size(); ++i)
            {
                accessor.maxValues.emplace_back(max[i]);
            }
        }
    };

    template <>
    inline uint gltf_file::getMinValue<uint>(const std::vector<uint> &values)
    {
        uint minx = UINT32_MAX;
        for (auto v : values)
        {
            if (v < minx)
                minx = v;
        }
        return minx;
    }
    template <>
    inline uint gltf_file::getMaxValue<uint>(const std::vector<uint> &values)
    {
        uint maxx = -UINT32_MAX;
        for (auto v : values)
        {
            if (v > maxx)
                maxx = v;
        }
        return maxx;
    }

    template <>
    inline void gltf_file::createBufferAccessor<pmp::vec3>(tinygltf::Accessor &accessor, std::vector<pmp::vec3> &data)
    {
        accessor.count = data.size();
        accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
        accessor.type = TINYGLTF_TYPE_VEC3;
        setMinMaxValues(accessor, data);
    }

    template <>
    inline void gltf_file::createBufferAccessor<pmp::TexCoord>(tinygltf::Accessor &accessor, std::vector<pmp::TexCoord> &data)
    {
        accessor.count = data.size();
        accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
        accessor.type = TINYGLTF_TYPE_VEC2;
        setMinMaxValues(accessor, data);
    }
    //        void createBufferAccessor(tinygltf::Model &model, tinygltf::Accessor& accessor, std::vector<ivec4> &data);
    template <>
    inline void gltf_file::createBufferAccessor<pmp::vec4>(tinygltf::Accessor &accessor, std::vector<pmp::vec4> &data)
    {
        accessor.count = data.size();
        accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
        accessor.type = TINYGLTF_TYPE_VEC4;
        setMinMaxValues(accessor, data);
    }

    template <>
    inline void gltf_file::createBufferAccessor<pmp::dvec3>(tinygltf::Accessor &accessor, std::vector<pmp::dvec3> &data)
    {
        accessor.count = data.size();
        accessor.componentType = TINYGLTF_COMPONENT_TYPE_DOUBLE;
        accessor.type = TINYGLTF_TYPE_VEC4;
        setMinMaxValues(accessor, data);
    }

    template <>
    inline void gltf_file::createBufferAccessor<float>(tinygltf::Accessor &accessor, std::vector<float> &data)
    {
        accessor.count = data.size();
        accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
        accessor.type = TINYGLTF_TYPE_VEC3;
        float min(std::numeric_limits<float>::max());
        float max(0);
        for (auto v : data)
        {
            if (v < min)
            {
                min = v;
            }
            if (v > max)
            {
                max = v;
            }
        }
        accessor.minValues = {min};
        accessor.maxValues = {max};
    }

    template <>
    inline void gltf_file::createBufferAccessor<gltf_ivec4>(tinygltf::Accessor &accessor, std::vector<gltf_ivec4> &data)
    {
        accessor.count = data.size();
        accessor.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE;
        accessor.type = TINYGLTF_TYPE_VEC4;

        // Min and max Values are not possible?
    }
    template <>
    inline void gltf_file::createBufferAccessor<uint>(tinygltf::Accessor &accessor, std::vector<uint> &data)
    {
        accessor.count = data.size();
        accessor.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT;
        accessor.type = TINYGLTF_TYPE_SCALAR;
        accessor.minValues = {(double)data[0]};
        accessor.maxValues = {(double)data[data.size() - 1]};

        // Min and max Values are not possible?
    }
    template <>
    inline void gltf_file::createBufferAccessor<pmp::mat4>(tinygltf::Accessor &accessor, std::vector<pmp::mat4> &data)
    {
        accessor.count = data.size();
        accessor.componentType = TINYGLTF_COMPONENT_TYPE_FLOAT;
        accessor.type = TINYGLTF_TYPE_MAT4;

        // Min and max Values are not possible?
    }
    template <>
    inline void gltf_file::createBufferAccessor<stbi_uc>(tinygltf::Accessor &accessor, std::vector<stbi_uc> &data)
    {
        accessor.count = data.size();
        accessor.componentType = TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE;
        accessor.type = TINYGLTF_TYPE_SCALAR;
        // Min and max Values are not possible?
    }

} // namespace pmp_character

#endif // GLTF_FILE_H
