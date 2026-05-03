// =====================================================================================================================

#pragma once
#ifndef PMPEXT_TRIANGLEKDTREE_H
#define PMPEXT_TRIANGLEKDTREE_H

// =====================================================================================================================

#include <pmp/surface_mesh.h>

#include <pmp/algorithms/distance_point_triangle.h>

#include "pmp_ext/defs.h"

// =====================================================================================================================

NAMESPACE_BEGIN(pmp_ext)

// =====================================================================================================================

class TriangleKDTree
{
public:
    TriangleKDTree(const pmp::SurfaceMesh* mesh,
                   unsigned int max_faces = 10, unsigned int max_depth = 30, const std::string& property = "v:point");

    ~TriangleKDTree() { delete root_; }

    struct NearestNeighbor
    {
        pmp::Scalar dist;
        pmp::Face face;
        pmp::Point nearest;
    };

    NearestNeighbor nearest(const pmp::Point& p) const;

private:
    using Faces = std::vector<pmp::Face>;

    struct Node
    {
        Node() = default;

        ~Node()
        {
            delete faces;
            delete left_child;
            delete right_child;
        }

        unsigned char axis;
        pmp::Scalar split;
        Faces* faces{nullptr};
        Node* left_child{nullptr};
        Node* right_child{nullptr};
    };

    void build_recurse(Node* node, unsigned int max_handles,
                       unsigned int depth);

    void nearest_recurse(Node* node, const pmp::Point& point,
                         NearestNeighbor& data) const;

    Node* root_;
    std::vector<std::array<pmp::Point, 3> > face_points_;
};

// =====================================================================================================================

NAMESPACE_END(pmp_ext)

// =====================================================================================================================

#endif //PMPEXT_TRIANGLEKDTREE_H

// =====================================================================================================================
