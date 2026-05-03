// =====================================================================================================================

#pragma once
#ifndef PMPEXT_BSPTREE_H
#define PMPEXT_BSPTREE_H

// =====================================================================================================================

#include <vector>

#include <pmp/types.h>

#include "pmp_ext/defs.h"
#include "pmp_ext/priority_queue.h"

// =====================================================================================================================

NAMESPACE_BEGIN(pmp_ext)

// =====================================================================================================================

/// \brief Data structure for fast nearest neighbour search. Implemented as a point based Binary Space Partition Tree
class BSPTree
{
public:
    //-------------------------------------------------------------- public types

    typedef std::vector<pmp::Point> Points;
    typedef Points::const_iterator ConstPointIter;


    // the elemnent stored in the tree
    struct Element
    {
        Element(const pmp::Point& _p, int _idx) : point(_p), idx(_idx)
        {
        }

        pmp::Point point;
        int idx;
    };

    typedef std::vector<Element> Elements;
    typedef Elements::iterator ElementIter;
    typedef Elements::const_iterator ConstElementIter;


    // Functor for partitioning wrt splitting plane
    struct PartPlane
    {
        PartPlane(unsigned char _cut_dim, pmp::Scalar _cut_val)
            : cut_dim_(_cut_dim), cut_val_(_cut_val)
        {
        }

        bool operator()(const Element& _e) const { return _e.point[cut_dim_] > cut_val_; }

        unsigned char cut_dim_;
        pmp::Scalar cut_val_;
    };


    // priority queue for kNN
    typedef MaxPriorityQueue<int, float> PrioQ;

    // Store nearest neighbor information
    struct NearestNeighborData
    {
        pmp::Point referencePoint, offset;
        pmp::Scalar dist;
        int nearest;
        unsigned int leaf_tests;
    };


    // Store kNN information
    struct kNearestNeighborData
    {
        pmp::Point ref, off;
        pmp::Scalar dist;
        PrioQ k_nearest;
        unsigned int leaf_tests;
    };


    // Store ball information
    struct ballData
    {
        pmp::Point ref, off;
        pmp::Scalar dist;
        unsigned int leaf_tests;
    };


    // Node of the tree: contains parent, children and splitting plane
    struct Node
    {
        Node(ElementIter _begin, ElementIter _end)
            : left_child_(0), right_child_(0), begin_(_begin), end_(_end)
        {
        }

        ~Node()
        {
            delete left_child_;
            delete right_child_;
        }

        Node* left_child_,* right_child_;
        ElementIter begin_, end_;
        unsigned char cut_dim_;
        pmp::Scalar cut_val_;
    };

public:
    //----------------------------------------------------------- public methods

    /** Constructor: need traits that define the types and
        give us the points by traits_.point(PointHandle) */
    BSPTree(const Points& _points) : points_(_points), root_(nullptr)
    {
    }

    /// Destructor
    ~BSPTree() { delete root_; }

    /// Build the tree. Returns number of nodes.
    unsigned int build(unsigned int _max_handles, unsigned int _max_depth);

    /// Return handle of the nearest neighbor
    NearestNeighborData nearest(const pmp::Point& _p) const;

    /// Return handles of the k nearest neighbors by inserting them
    /// into an container using _inserter (e.g. std::back_inserter)
    int k_nearest(const pmp::Point& _p, unsigned int _k, std::vector<int>& _knn) const;

    /// Return handles of the neighbors within a ball centered at p by
    /// inserting them into a container
    int ball(const pmp::Point& p, pmp::Scalar radius, std::vector<int>& ball) const;

private:
    //----------------------------------------------------------- private methods

    // Recursive part of build()
    void _build(Node* _node,
                unsigned int _max_handles,
                unsigned int _depth);

    // Recursive part of nearest()
    void _nearest(Node* _node, pmp::Scalar rd, NearestNeighborData& _data) const;


    // Recursive part of k_nearest()
    void _k_nearest(Node* _node, pmp::Scalar _rd, kNearestNeighborData& _data) const;


    // Recursive part of ball()
    void _ball(Node* _node, pmp::Scalar _rd, ballData& _data, pmp::Scalar radius_sq, std::vector<int>& ball) const;


    //-------------------------------------------------------------- private data

    const Points& points_;
    Elements elements_;
    Node* root_;
    unsigned int n_nodes_;
};

// =====================================================================================================================

NAMESPACE_END(pmp_ext)

// =====================================================================================================================

#endif //PMPEXT_BSPTREE_H

// =====================================================================================================================
