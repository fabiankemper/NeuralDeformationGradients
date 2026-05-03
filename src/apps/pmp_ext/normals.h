// =====================================================================================================================
// Created by fabian on 8/23/24.
// mail@fabiankemper.de
// =====================================================================================================================

#pragma once
#ifndef VOLUMETRIC_CHARACTER_NORMALS_H
#define VOLUMETRIC_CHARACTER_NORMALS_H

// =====================================================================================================================

#include <pmp/surface_mesh.h>

#include "volanim_utils/defs.h"

// =====================================================================================================================

NAMESPACE_BEGIN(pmp_ext)
NAMESPACE_BEGIN(normals)

// =====================================================================================================================

using Scalar = pmp::Scalar;
using Point = pmp::Point;
using Normal = pmp::Normal;
using SurfaceMesh = pmp::SurfaceMesh;

using Vertex = pmp::Vertex;
using Halfedge = pmp::Halfedge;
using Edge = pmp::Edge;
using Face = pmp::Face;

// =====================================================================================================================

//! \brief Compute vertex normals for the whole \p mesh.
//! \details Calls vertex_normal() for each vertex and adds a new
//! vertex property of type Normal named "v:normal".
//! \note This algorithm works on general polygon meshes.
//! \ingroup algorithms
void vertex_normals(SurfaceMesh& mesh, const std::string& vpoint_property = "v:point", const std::string& vnormal_property = "v:normal");

//! \brief Compute face normals for the whole \p mesh.
//! \details Calls face_normal() for each face and adds a new face
//! property of type Normal named "f:normal".
//! \note This algorithm works on general polygon meshes.
//! \ingroup algorithms
void face_normals(SurfaceMesh& mesh, const std::string& vpoint_property = "v:point", const std::string& fnormal_property = "f:normal");

Normal vertex_normal(const SurfaceMesh& mesh, Vertex v, const std::string& property = "v:point");

Normal face_normal(const SurfaceMesh& mesh, Face f, const std::string& property = "v:point");

// =====================================================================================================================

NAMESPACE_END(normals)
NAMESPACE_END(pmp_ext)

// =====================================================================================================================

#endif //VOLUMETRIC_CHARACTER_NORMALS_H

// =====================================================================================================================
