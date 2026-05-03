//
// Created by friedemannr on 04.12.23.
//

#pragma once

#include <pmp/surface_mesh.h>

#include "Blendshapes.h"
#include "node_types/SceneNode.h"

namespace pmp_character
{
    /// \brief Node representing submesh of a character
    class MeshNode : public SceneNode
    {
    public:
        MeshNode();
        ~MeshNode();
        /// The associated pmp SurfaceMesh
        pmp::SurfaceMesh mesh;
        pmp::Mat4<pmp::Scalar> transform;
        /// Name of the Texture associated with the BaseColor. Subject to Change for PBR Support
        std::string texture_name = "";
        pmp_character::BlendshapeCollection blendshape_collection;
    };
} // namespace pmp_character
