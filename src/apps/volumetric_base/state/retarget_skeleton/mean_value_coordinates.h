// =====================================================================================================================

#pragma once
#ifndef MEAN_VALUE_COORDINATES_H
#define MEAN_VALUE_COORDINATES_H

// =====================================================================================================================

#include <pmp_character/Skeleton.h>

#include "volanim_utils/defs.h"

#include "volumetric_base/aliases.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(mvc)

// =====================================================================================================================

using Skeleton = pmp_character::Skeleton;
using SurfaceMesh = pmp::SurfaceMesh;
using MeanValueCoords = std::vector<double>;
static std::string MVC_PROPERTY_NAME = "v:mvc_weights";

// =====================================================================================================================

/// \brief Computes weights for mean value coordinates for a set of points dependent on the vertices of the mesh.
/// Adds a vertex property to the mesh
/// \param mesh Mesh Vertices
/// \param points [out] The points for the MVC
/// \param pos_prop name of the position property for calculating weights
auto compute_weights(
    const SurfaceMesh& mesh,
    const std::vector<pmp::Point>& points,
    const std::string& pos_prop = "v:point"
) -> void;


/// \brief Wrapper method for generating MVC for a given character
/// \param skeleton aniamation skeleton
/// \param mesh skinned mesh
/// \param pos_prop name of the position property for calculating weights
/// \return bool if it was successful
auto compute_weights_from_current_joint_positions(
    const Skeleton& skeleton,
    SurfaceMesh& mesh,
    const std::string& pos_prop = "v:point"
) -> void;

/// \brief Recomputes the joint positions based on the old MVC and a new mesh
/// \param mesh
/// \param points [out] new positions for joints
/// \param pos_prop name of the position property for calculating new joint positions
/// \return
auto compute_new_joint_positions(
    const pmp::SurfaceMesh& mesh,
    std::vector<pmp::Point>& points,
    const std::string& pos_prop = "v:point"
) -> void;

/// \brief Recalculates the Joint Positions of a character given a new mesh/new mesh positions
/// \param skeleton
/// \param mesh
auto recalculate_joint_positions(const Skeleton& skeleton, const SurfaceMesh& mesh) -> std::vector<pmp::vec3>;


/// \brief Transfer mean value coord property to new mesh from old mesh
/// \param skeleton Rig to recalculate joint lengths for
/// \param source_mesh surface mesh matching current joint positions
/// \param target_mesh surface mesh in source topology for new joint positions
auto fit_rig_to_mesh(Skeleton& skeleton, const SurfaceMesh& source_mesh, const SurfaceMesh& target_mesh) -> void;

// =====================================================================================================================

NAMESPACE_END(mvc)
NAMESPACE_END(volumetric_base)

// =====================================================================================================================

#endif //MEAN_VALUE_COORDINATES_H

// =====================================================================================================================
