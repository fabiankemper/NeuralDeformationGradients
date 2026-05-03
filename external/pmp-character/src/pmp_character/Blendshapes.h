//
// Created by peasman on 11.12.23.
//

#ifndef BLENDSHAPE_H
#define BLENDSHAPE_H
#pragma once
#include <map>
#include <set>

#include <pmp/mat_vec.h>
#include <pmp/surface_mesh.h>

#include "PropertyNames.h"

namespace pmp_character
{
    /// \brief Class for representing a single Blendshape Key for a single MeshNode
    class Blendshape
    {
    public:
        Blendshape(std::string _name) : name(_name) {}
        /// \brief Name of the blendshape/Key
        std::string name;
        std::vector<pmp::Vertex> movingVertices;
        std::vector<pmp::Vertex> movingNormals;
        std::vector<pmp::Point> deformation;
        std::vector<pmp::Point> normalDeformation;

        void applyBlendshape(pmp::SurfaceMesh &mesh, float weight)
        {
            auto blendpos = mesh.get_vertex_property<pmp::Point>(properties::vertexBlendPosition);
            auto blend_normal = mesh.get_vertex_property<pmp::Point>(properties::vertexBlendNormal);
            // Deformation per Mesh
            for (size_t i = 0; i < movingVertices.size(); i++)
            {
                auto vertex = movingVertices[i];
                blendpos[vertex] += weight * deformation[i];
            }
            for (size_t i = 0; i < movingNormals.size(); i++)
            {
                auto vertex = movingNormals[i];
                blend_normal[vertex] += weight * normalDeformation[i];
            }
        }
    };

    /// \brief Wrapper Class for Blendshape Vector maybe delete??
    class BlendshapeCollection
    {
    public:
        BlendshapeCollection();
        ~BlendshapeCollection() = default;
        std::vector<Blendshape> blendshapes;
        bool empty = false;
    };

} // namespace pmp_character


#endif // BLENDSHAPE_H
