// =====================================================================================================================

#pragma once
#ifndef VOLUMETRIC_ANIMATION_SKINNABLE_MESH_H
#define VOLUMETRIC_ANIMATION_SKINNABLE_MESH_H

// =====================================================================================================================

#include <pmp/surface_mesh.h>

#include "volumetric_base/aliases.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)

// =====================================================================================================================

using Rig = pmp_character::Skeleton;
namespace properties = pmp_character::properties;

// =====================================================================================================================

class SkinnableMesh
{
public:
    SkinnableMesh();
    SkinnableMesh(const SkinnableMesh& other);
    explicit SkinnableMesh(const SurfaceMesh& mesh);

    auto operator=(const SkinnableMesh& other) -> SkinnableMesh&;

    virtual ~SkinnableMesh() = default;

    bool operator==(const SkinnableMesh& other) const;

    // no skinnign, just copy vertices for view buffer
    auto null_skinning() -> void;

    // getter & setter
    auto mesh() -> SurfaceMesh& { return _mesh; }
    [[nodiscard]]
    auto mesh() const -> const SurfaceMesh& { return _mesh; }
    auto set_mesh(const SurfaceMesh& mesh) -> void;

    // get a baked mesh, all skinning is baked fixed in positions
    [[nodiscard]]
    auto bake_mesh(const std::string& pos_property_name = properties::vertexGlobalPosition) const -> SurfaceMesh;
    auto skinning_from_baked_mesh(const SurfaceMesh& baked_mesh, const std::string& pos_property_name = properties::vertexGlobalPosition) -> void;
    auto positions_from_baked_mesh(const SurfaceMesh& baked_mesh) -> void;

    // apply skinning from different mesh
    auto copy_skinning_from_skinnable_mesh(const SkinnableMesh& other) -> void;

    // helper
    [[nodiscard]]
    auto is_dirty() const -> bool { return _dirty.load(); }
    auto set_dirty(const bool& dirty) -> void { _dirty.store(dirty); }

    // skinning prepare
    auto skinning_setup(const Rig* rig) -> void;

    // access internal matrices
    [[nodiscard]]
    auto linear_skinning_matrices() -> std::vector<pmp::mat4>&;

    [[nodiscard]]
    auto dualquat_skinning_matrices() -> std::vector<pmp_character::vec8>&;

    [[nodiscard]]
    auto transform() -> pmp::mat4&;

    [[nodiscard]]
    auto has_skinning_weights() const -> bool;
    auto set_has_skinning_weights(bool value) -> void;
    // skinning mesh
    pmp::SurfaceMesh _mesh;
protected:

    /// skinning matrices for linear blend skinning
    std::vector<pmp::mat4> _skinning_matrices;
    /// skinning dual quaternions for dual quaternion skinning
    std::vector<pmp_character::vec8> _skinning_dualquats;
    // global transformation
    pmp::mat4 _transform = pmp::mat4::identity();

    // mark openGL buffer outdated
    std::atomic<bool> _dirty = true;

    // save if this mesh has REASONABLE skinning parameters set
    bool _has_skinning_weights;
};


// =====================================================================================================================

NAMESPACE_END(volumetric_base)

// =====================================================================================================================

#endif //VOLUMETRIC_ANIMATION_SKINNABLE_MESH_H

// =====================================================================================================================
