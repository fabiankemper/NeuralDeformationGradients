#pragma once

#include <pmp/mat_vec.h>
#include <pmp/types.h>

namespace pmp_character {

    //! template specialization for a vector of eight float values
    typedef pmp::Vector<float, 8> vec8;
    //! template specialization for a vector of eight double values
    typedef pmp::Vector<double, 8> dvec8;
    //! template specialization for a vector of eight bool values
    typedef pmp::Vector<bool, 8> bvec8;
    //! template specialization for a vector of eight int values
    typedef pmp::Vector<int, 8> ivec8;
    //! template specialization for a vector of eight unsigned int values
    typedef pmp::Vector<unsigned int, 8> uvec8;


}

typedef pmp::Mat4<pmp::Scalar> M4x4;
typedef pmp::Mat3<pmp::Scalar> M3x3;
typedef pmp::Vector<pmp::Scalar,4> V4;
typedef pmp::Vector<pmp::Scalar,3> V3;


enum struct NodeType
{
    Mesh, SkinnedMesh, Character, Node, PointSet
};


inline std::string toString(NodeType type) {
    switch (type) {
    case NodeType::Mesh: return "Mesh";
    case NodeType::SkinnedMesh: return "SkinnedMesh";
    case NodeType::Character: return "Character";
    case NodeType::Node: return "Node";
    case NodeType::PointSet: return "PointSet";
    default: return "Unknown";
    }
}