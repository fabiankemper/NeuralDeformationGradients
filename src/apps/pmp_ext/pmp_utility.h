// =====================================================================================================================

#pragma once
#ifndef PMPEXT_PMPUTILITY_H
#define PMPEXT_PMPUTILITY_H

// =====================================================================================================================

#include <pmp/surface_mesh.h>
#include <pmp/mat_vec.h>
#include <pmp_character/node_types/Character.h>
#include <pmp_character/node_types/PointSet.h>

#include "pmp_ext/defs.h"

// =====================================================================================================================

NAMESPACE_BEGIN(pmp_ext::utils)

// =====================================================================================================================

/// Returns a vector with the per coordinate smallest values
/// @param ref
/// @param o
void minimizeVectorCoordinates(pmp::vec3& ref, const pmp::vec3& o);

/// Returns a vector with the per coordinate biggest values
/// @param ref
/// @param o
void maximizeVectorCoordinates(pmp::vec3& ref, const pmp::vec3& o);

/// Centers a template to the mass center of a pointcloud
/// @param character
/// @param point_set
void meanCenterTemplateToPointcloud(const pmp_character::Character& character, pmp_character::PointSet& point_set);

/// Gets the vertices of a face of a mesh in a std::vector<pmp::vec3>
/// @param mesh The mesh
/// @param face The face
/// @param vertices out parameter for vertices. Needs to be initialized outside
void getFaceVertices(const pmp::SurfaceMesh& mesh, pmp::Face face, std::vector<pmp::Vertex>& vertices);

/// Gets the vertex positions of a face with the given property. You can get the skinned positions by providing
/// property = "v:skinnedPosition" or similar
/// @param mesh The mesh
/// @param face The face
/// @param vertices The vector where coordinates get stored
/// @param property The name of the property where the positions are received from
void getFaceVertexPositions(const pmp::SurfaceMesh& mesh, pmp::Face face, std::vector<pmp::vec3>& vertices,
                            const std::string& property = pmp_character::properties::vertexPosition);

/// Loads a vertexProperty of type double out of a file where in every line there is a value corresponding to a
/// vertex
/// @param propertyPath Path to file with values stored
/// @param mesh The mesh
/// @param propertyName_ Name of the property
/// @return std::vector<double> with all loaded values;
std::vector<double> loadVertexProperty(const std::filesystem::path& propertyPath, pmp::SurfaceMesh& mesh,
                                       std::string propertyName_);

/// Special case of @link loadVertexProperty where the vertex weights for fitting are loaded
/// @param weightsPath
/// @param mesh
/// @return
std::vector<double> loadVertexWeights(const std::filesystem::path& weightsPath, pmp::SurfaceMesh& mesh);

/// Special case of @link loadVertexProperty where the laplace vertex weights for fitting are loaded
/// @param weightsPath
/// @param mesh
/// @return
std::vector<double> loadLaplaceWeights(const std::filesystem::path& weightsPath, pmp::SurfaceMesh& mesh);

/// Flip the UV Coordinates. This is needed as Metashape uses a different UV Coordinate set
/// @param mesh
void flipUVCoordinates(const pmp::SurfaceMesh& mesh);

void transformVec3Property(pmp::SurfaceMesh& mesh, const pmp::mat4& transform, const std::string& propertyName);

double edgeAreaOld(pmp::SurfaceMesh& mesh, pmp::Edge e);

void addHalfEdgeCotanAttribute(pmp::SurfaceMesh& mesh);

void addEdgeLaplacianAttribute(pmp::SurfaceMesh& mesh);


// =====================================================================================================================

NAMESPACE_END(pmp_ext::utils)

// =====================================================================================================================

#endif //PMPEXT_PMPUTILITY_H

// =====================================================================================================================
