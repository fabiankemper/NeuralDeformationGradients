//
// Created by fabian on 3/31/23.
//

#ifndef TORCH3D_VIEWER_TRITRIINTERSECT_H
#define TORCH3D_VIEWER_TRITRIINTERSECT_H

// static compilation

#define IGL_INLINE
//#define IGL_INLINE inline
#include "Eigen/Core"

namespace igl {


// Three-dimensional Triangle-Triangle overlap test
//   if triangles are co-planar
//
// Input:
//   p1,q1,r1  - vertices of the 1st triangle (3D)
//   p2,q2,r2  - vertices of the 2nd triangle (3D)
//
// Output:
//
//   Return true if two triangles overlap
template <typename DerivedP1,typename DerivedQ1,typename DerivedR1,
    typename DerivedP2,typename DerivedQ2,typename DerivedR2>
IGL_INLINE bool tri_tri_overlap_test_3d(
        const Eigen::MatrixBase<DerivedP1> &  p1,
        const Eigen::MatrixBase<DerivedQ1> &  q1,
        const Eigen::MatrixBase<DerivedR1> &  r1,
        const Eigen::MatrixBase<DerivedP2> &  p2,
        const Eigen::MatrixBase<DerivedQ2> &  q2,
        const Eigen::MatrixBase<DerivedR2> &  r2);


// Three-dimensional Triangle-Triangle Intersection Test
// additionally computes the segment of intersection of the two triangles if it exists.
// coplanar returns whether the triangles are coplanar,
// source and target are the endpoints of the line segment of intersection
//
// Input:
//   p1,q1,r1  - vertices of the 1st triangle (3D)
//   p2,q2,r2  - vertices of the 2nd triangle (3D)
//
// Output:
//   coplanar - flag if two triangles are coplanar
//   source - 1st point of intersection (if exists)
//   target - 2nd point in intersection (if exists)
//
//   Return true if two triangles intersect
    template <typename DerivedP1,typename DerivedQ1,typename DerivedR1,
        typename DerivedP2,typename DerivedQ2,typename DerivedR2,
        typename DerivedS,typename DerivedT>
    IGL_INLINE bool tri_tri_intersection_test_3d(
        const Eigen::MatrixBase<DerivedP1> & p1, const Eigen::MatrixBase<DerivedQ1> & q1_, const Eigen::MatrixBase<DerivedR1> & r1_,
        const Eigen::MatrixBase<DerivedP2> & p2_, const Eigen::MatrixBase<DerivedQ2> & q2_, const Eigen::MatrixBase<DerivedR2> & r2_,
        bool & coplanar,
        Eigen::MatrixBase<DerivedS> & source,
        Eigen::MatrixBase<DerivedT> & target);



// Two-dimensional Triangle-Triangle Overlap Test
// Input:
//   p1,q1,r1  - vertices of the 1st triangle (2D)
//   p2,q2,r2  - vertices of the 2nd triangle (2D)
//
// Output:
//   Return true if two triangles overlap
template <typename DerivedP1,typename DerivedQ1,typename DerivedR1,
    typename DerivedP2,typename DerivedQ2,typename DerivedR2>
IGL_INLINE bool tri_tri_overlap_test_2d(
        const Eigen::MatrixBase<DerivedP1> &p1, const Eigen::MatrixBase<DerivedQ1> &q1, const Eigen::MatrixBase<DerivedR1> &r1,
        const Eigen::MatrixBase<DerivedP2> &p2, const Eigen::MatrixBase<DerivedQ2> &q2, const Eigen::MatrixBase<DerivedR2> &r2);


// SIMPLE CHECK
template <typename DerivedP1, typename DerivedP2>
IGL_INLINE bool check_same(
    const Eigen::MatrixBase<DerivedP1> &p1, const Eigen::MatrixBase<DerivedP2> &p2);

};



// =====================================================================================================================
#ifndef IGL_STATIC_LIBRARY
#  include "TriTriIntersect.cpp"
#endif

// =====================================================================================================================


#endif //TORCH3D_VIEWER_TRITRIINTERSECT_H
