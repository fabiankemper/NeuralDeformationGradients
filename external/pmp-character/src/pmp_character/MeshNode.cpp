//
// Created by peasman on 06.12.23.
//
#include "MeshNode.h"

#include "node_types/Character.h"
#include "DualQuaternion.h"
#include "PropertyNames.h"
namespace pmp_character
{
    /// Constructor creates Vertex Attributes for the surface mesh
    MeshNode::MeshNode() : SceneNode("")
    {
        mesh.add_vertex_property<pmp::Normal>(properties::vertexNormals);
        mesh.add_vertex_property<pmp::TexCoord>(properties::vertexTextureCoordinates);
        mesh.add_vertex_property<vec8>(properties::vertexSkinningWeights);
        mesh.add_vertex_property<ivec8>(properties::vertexSkinningDepends);
        mesh.add_vertex_property<pmp::vec3>(properties::vertexGlobalPosition);
        mesh.add_vertex_property<pmp::vec3>(properties::vertexGlobalNormal);
        nodeType = NodeType::SkinnedMesh;
    }

    MeshNode::~MeshNode() {}
} // namespace pmp_character
