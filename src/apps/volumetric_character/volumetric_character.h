// =====================================================================================================================

#pragma once
#ifndef PMPCHARACTER_VOLUMETRIC_CHARACTER_H
#define PMPCHARACTER_VOLUMETRIC_CHARACTER_H

// =====================================================================================================================

#include <ostream>

#include <pmp/bounding_box.h>
#include <pmp/surface_mesh.h>
#include <pmp_character/node_types/Character.h>

#include "volumetric_base/layered_mesh.h"
#include "volumetric_base/skinnable_mesh.h"
#include "volumetric_base/volumetric_types.h"
#include "volumetric_base/state/configuration.h"
#include "volumetric_base/state/volumetric_state.h"

#include "volumetric_skinning/base/skinning_base.h"

#include "../volumetric_base/diffgeo/diffgeo.h"
#include "volumetric_skinning/bary_warp/bary_warp.h"
#include "volumetric_skinning/bary_warp/barycentric_warp.h"

// =====================================================================================================================
// class declarations

class PrismBasedCorotationalElasticity;
class PerVertexElasticity;
class tetrahedral_projective_skinner;

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_character)

// =====================================================================================================================

// aliases
using Path = std::filesystem::path;

using Character = pmp_character::Character;
using SurfaceMesh = pmp::SurfaceMesh;

using Layer = volumetric_base::Layer;
using SkinnableMesh = volumetric_base::SkinnableMesh;
using VolumetricState = volumetric_base::VolumetricState;

using DiffgeoSolver = volumetric_base::diffgeo::DifferentialGeometrySolver;

using VertexMap = std::map<unsigned int, unsigned int>;

// =====================================================================================================================
// constants

// =====================================================================================================================

struct VolIOFlags
{
    bool skin = true; // write skin layer
    // hires layers
    bool hires_musc = true;
    bool hires_skel = true;
    // wraps
    bool musc_wrap = false;
    bool skel_wrap = false;
};

// =====================================================================================================================

/**
 * Note:
 *
 * Current state for volumetric character
 * Use the "this" surface property for the current skin surface mesh. We load skinning weights as vertex property
 *
 * I currently use different sub-meshes for SKEL, MUSC and SKIN surfaces (in deformed and normal view)
 * This will be changed in a future version.
 */

class VolumetricCharacter
{
public:
    // constructors
    VolumetricCharacter();
    VolumetricCharacter(const VolumetricCharacter& other);
    auto operator=(const VolumetricCharacter& other) -> VolumetricCharacter&;
    virtual ~VolumetricCharacter();

    // sub-structs
    enum struct SkinningMode {
        NO_SKINNING = -1,
        LINEAR_BLEND_SKINNING = 0,
        DUAL_QUATERNION_SKINNING = 1,
        VOLUMETRIC_SKINNING = 2,
        VOLUMETRIC_PRISM_SKINNING = 3,
        ELASTIC_SKINNING = 4,
        PROJECTIVE_SKINNING = 5,
        ELASTIC_BLENDSHAPE = 6,
        NN_VERTEX_SKINNING = 7,
        TETRAHEDRON_PROJECTIVE_SKINNING = 8,
        TETRAHEDRON_ELASTIC_SKINNING = 9,
        DEFGRAD_NN_BLENDSHAPE = 10,
        NN_DEFGRAD_SKINNING = 11,
        NONLINEAR_ELASTICITY = 12,
        NEO_HOOKEAN = 15,
        VERTEX_BLENDSHAPE = 13,
        RBF_WARP = 14, // RBF Warp skinner for Skeleton
    };

    // re-create skinnable meshes from state
    auto reinit_skinnable_layers() -> void;

    // access animation layers
    auto skinnable_layer(const Layer& layer) -> SkinnableMesh&;
    [[nodiscard]]
    auto skinnable_layer(const Layer& layer) const -> const SkinnableMesh&;

    auto highres_layer(const Layer& layer) -> SkinnableMesh&;
    [[nodiscard]]
    auto highres_layer(const Layer& layer) const -> const SkinnableMesh&;

    // get bounding box
    auto bounds() -> pmp::BoundingBox;

    // loading methods
    auto set_view_layers(const SurfaceMesh& skel_mesh, const SurfaceMesh& musc_mesh, const SurfaceMesh& skin_mesh) -> void;

    // create skinnable mesh layers, first method to be called
    auto updated_template_layers() -> void;

    // update internal skinning parameters from state
    auto updated_skinning_parameters() -> void;

    // create and set mean value coordinates on template skeleton wrap
    auto update_mvc() -> void;

    // set mean meshes for deformation transfer
    auto updated_mean_meshes() -> void;

    // update skinning
    auto skinning(bool converge = false) -> void;

    auto set_new_skinner() -> void;
    auto set_skinning_mode(SkinningMode skinning_mode) -> void;

    // set joint angles
    auto set_pose(const std::vector<float>& joint_angles) -> void;

    // set translation
    auto set_translation(std::array<float, 3> translation) -> void;

    // trigger buffer update
    auto set_dirty(bool dirty = true) -> void;
    auto set_skeleton_dirty(bool dirty = true) -> void;
    [[nodiscard]]
    auto is_skeleton_dirty() const -> bool { return _skeleton_dirty.load(); }

    auto set_highres_dirty(bool dirty = true) -> void;
    [[nodiscard]]
    auto is_highres_dirty() const -> bool { return _highres_dirty.load(); }

    // set data object
    auto set_state(VolumetricState* state) -> void;
    [[nodiscard]]
    auto state() const -> VolumetricState*;


    void update_animators();
    // mark layers updated
    auto update_layers() -> void;

    // perform gui operations from skinner
    auto imgui_skinner() const -> void;

    // current skinner
    volumetric_skinning::SkinningBase* _skinner = nullptr;
    std::string defgrad_weight_path;
    std::string vertex_weight_path;
    // rendering
    SkinningMode _skinning_mode = SkinningMode::LINEAR_BLEND_SKINNING;

    auto update_use_mean_mesh() -> void;
    bool use_template_mesh = false;
    std::map<SkinningMode, volumetric_skinning::SkinningBase*> skinning_cache;

    // enable high-res mesh calculation
    bool highres_skinning = true;

    // calculate high-res anatomy
    auto calculate_high_res_anatomy_rbf(long n_rbf_kernels = 700) -> void;

    // save meshes
    auto save_meshes(const std::filesystem::path& out_dir, const std::string& prefix, VolIOFlags flags = VolIOFlags{}) -> void;

    bary_warp::BarycentricWarp _hires_skinner {};
    float current_fps = 60.0F;

protected:
    // bindpose angles
    std::vector<float> _bindpose_joint_angles;

    std::atomic<bool> _skeleton_dirty = true;
    std::atomic<bool> _layers_dirty = true;
    std::atomic<bool> _highres_dirty = true;

    // layers
    VolumetricState* _state = nullptr;

    // final skinnable meshes, contains v:depends and v:weight properties (currently)
    // todo: use v:depends and v:weight from _volumetric_data object
    std::array<SkinnableMesh, 3> _skinnable_meshes;

    // high-res anatomy meshes
    std::array<SkinnableMesh, 3> _highres_meshes;

    // re-calculate animation skeleton positions
    auto recalculate_joints() const -> void;
};

// =====================================================================================================================

inline std::ostream& operator<<(std::ostream& os, VolumetricCharacter::SkinningMode mode) {
    switch (mode) {
        case VolumetricCharacter::SkinningMode::NO_SKINNING:                   return os << "NO_SKINNING";
        case VolumetricCharacter::SkinningMode::LINEAR_BLEND_SKINNING:         return os << "LINEAR_BLEND_SKINNING";
        case VolumetricCharacter::SkinningMode::DUAL_QUATERNION_SKINNING:      return os << "DUAL_QUATERNION_SKINNING";
        case VolumetricCharacter::SkinningMode::VOLUMETRIC_SKINNING:           return os << "VOLUMETRIC_SKINNING";
        case VolumetricCharacter::SkinningMode::VOLUMETRIC_PRISM_SKINNING:     return os << "VOLUMETRIC_PRISM_SKINNING";
        case VolumetricCharacter::SkinningMode::ELASTIC_SKINNING:              return os << "ELASTIC_SKINNING";
        case VolumetricCharacter::SkinningMode::PROJECTIVE_SKINNING:           return os << "PROJECTIVE_SKINNING";
        case VolumetricCharacter::SkinningMode::ELASTIC_BLENDSHAPE:            return os << "ELASTIC_BLENDSHAPE";
        case VolumetricCharacter::SkinningMode::NN_VERTEX_SKINNING:            return os << "NN_VERTEX_SKINNING";
        case VolumetricCharacter::SkinningMode::TETRAHEDRON_PROJECTIVE_SKINNING:return os << "TETRAHEDRON_PROJECTIVE_SKINNING";
        case VolumetricCharacter::SkinningMode::TETRAHEDRON_ELASTIC_SKINNING:  return os << "TETRAHEDRON_ELASTIC_SKINNING";
        case VolumetricCharacter::SkinningMode::DEFGRAD_NN_BLENDSHAPE:         return os << "DEFGRAD_NN_BLENDSHAPE";
        case VolumetricCharacter::SkinningMode::NN_DEFGRAD_SKINNING:           return os << "NN_DEFGRAD_SKINNING";
        case VolumetricCharacter::SkinningMode::NONLINEAR_ELASTICITY:          return os << "NONLINEAR_ELASTICITY";
        case VolumetricCharacter::SkinningMode::NEO_HOOKEAN:                   return os << "NEO_HOOKEAN";
        case VolumetricCharacter::SkinningMode::VERTEX_BLENDSHAPE:             return os << "VERTEX_BLENDSHAPE";
        case VolumetricCharacter::SkinningMode::RBF_WARP:                      return os << "RBF_WARP";
        default:                                                               return os << "UNKNOWN(" << int(mode) << ")";
    }
}

// =====================================================================================================================

NAMESPACE_END(volumetric_character)

// =====================================================================================================================

#endif //PMPCHARACTER_VOLUMETRIC_CHARACTER_H

// =====================================================================================================================
