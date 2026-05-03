// =====================================================================================================================

#include "skinnable_mesh.h"

#include <pmp_character/DualQuaternion.h>

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

SkinnableMesh::SkinnableMesh()
    : _mesh(SurfaceMesh())
{
    _has_skinning_weights = false;
    set_mesh(_mesh);
}

// ---------------------------------------------------------------------------------------------------------------------

SkinnableMesh::SkinnableMesh(const SkinnableMesh& other)
    : _mesh(SurfaceMesh()) // do not use _mesh(mesh) here, because we override this mesh later
{
    set_mesh(other._mesh);
    _has_skinning_weights = other.has_skinning_weights();
    _dirty.store(true);
}

// ---------------------------------------------------------------------------------------------------------------------

SkinnableMesh::SkinnableMesh(const SurfaceMesh& mesh)
    : _mesh(SurfaceMesh()) // do not use _mesh(mesh) here, because we override this mesh later
{
    _has_skinning_weights = false;
    set_mesh(mesh);
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinnableMesh::operator=(const SkinnableMesh& other) -> SkinnableMesh&
{
    if (this != &other) {
        _mesh = other._mesh;
        _has_skinning_weights = other._has_skinning_weights;
        _dirty.store(true);
    }

    return *this;
}

// ---------------------------------------------------------------------------------------------------------------------

bool SkinnableMesh::operator==(const SkinnableMesh& other) const
{
    return &this->_mesh == &other._mesh;
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinnableMesh::null_skinning() -> void
{
    auto global_pos = _mesh.get_vertex_property<pmp::vec3>(properties::vertexGlobalPosition);
    for (const auto vertex : _mesh.vertices()) {
        global_pos[vertex] = pmp::affine_transform(_transform, _mesh.position(vertex));
    }

    set_dirty(true);
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinnableMesh::set_mesh(const SurfaceMesh& mesh) -> void
{
    _mesh = mesh;

    // add normal property
    if (not _mesh.has_vertex_property(properties::vertexNormals)) {
        _mesh.add_vertex_property<pmp::Normal>(properties::vertexNormals);
    }

    // check if the mesh already had skinning weights
    if (_mesh.has_vertex_property(properties::vertexSkinningDepends) and _mesh.has_vertex_property(properties::vertexSkinningWeights)) {
        _has_skinning_weights = true;
    }

    // add skinning weights
    if (not _mesh.has_vertex_property(properties::vertexSkinningDepends)) {
        _mesh.add_vertex_property<pmp_character::ivec8>(properties::vertexSkinningDepends);
    }
    if (not _mesh.has_vertex_property(properties::vertexSkinningWeights)) {
        _mesh.add_vertex_property<pmp_character::vec8>(properties::vertexSkinningWeights);
    }

    // add skinning properties
    if (not _mesh.has_vertex_property(properties::vertexSkinnedPos)) {
        _mesh.add_vertex_property<pmp::Point>(properties::vertexSkinnedPos);
    }
    if (not _mesh.has_vertex_property(properties::vertexSkinnedNormal)) {
        _mesh.vertex_property<pmp::Normal>(properties::vertexSkinnedNormal);
    }
    if (not _mesh.has_vertex_property(properties::vertexGlobalPosition)) {
        _mesh.vertex_property<pmp::Point>(properties::vertexGlobalPosition);
    }
    if (not _mesh.has_vertex_property(properties::vertexGlobalNormal)) {
        _mesh.vertex_property<pmp::Normal>(properties::vertexGlobalNormal);
    }

    set_dirty(true);
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinnableMesh::bake_mesh(const std::string& pos_property_name) const -> SurfaceMesh
{
    SurfaceMesh result;

    // copy connectivity and reserve position space
    result.assign(_mesh);
    auto pos_property = _mesh.get_vertex_property<pmp::Point>(pos_property_name);

    // apply skinned positions to mesh
    for (const auto vertex : result.vertices()) {
        result.position(vertex) = pos_property[vertex];
    }
    return result;
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinnableMesh::skinning_from_baked_mesh(const SurfaceMesh& baked_mesh, const std::string& pos_property_name) -> void
{
    if (_mesh.n_vertices() != bake_mesh().n_vertices()) {
        std::cerr << "Cannot copy vertices from baked mesh. Vertex count do not match.\n";
    }

    if (not _mesh.has_vertex_property(pos_property_name)) {
        std::cerr << "[ERROR] Could not apply baked positions to mesh. Reason: No " + pos_property_name + " property.\n";
        return;
    }

    // copy positions to skinned vertices, used for external skinning
    auto pos_property = _mesh.vertex_property<pmp::Point>(pos_property_name);

    // apply skinned positions to mesh
    for (const auto vertex : baked_mesh.vertices()) {
        pos_property[vertex] = baked_mesh.position(vertex);
    }

    set_dirty(true);
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinnableMesh::positions_from_baked_mesh(const SurfaceMesh& baked_mesh) -> void
{
    if (_mesh.n_vertices() != baked_mesh.n_vertices()) {
        std::cerr << "Cannot copy vertices from baked mesh. Vertex count do not match.\n";
    }

    // copy position from sample to this positons
    for (auto const vertex : baked_mesh.vertices()) {
        _mesh.position(vertex) = baked_mesh.position(vertex);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinnableMesh::copy_skinning_from_skinnable_mesh(const SkinnableMesh& other) -> void
{
    if (other._mesh.n_vertices() != _mesh.n_vertices()) {
        std::cerr << "[ERROR] SkinnableMesh::copy_skinning_from_skinnable_mesh: Other mesh has different number of vertices.\n";
        return;
    }
    if (*this == other) {
        return;
    }

    // === copy vertex properties
    auto n_vertices = _mesh.n_vertices();

    // vertex skinned pos
    auto other_skinned_pos = other._mesh.get_vertex_property<pmp::Point>(properties::vertexSkinnedPos);
    auto skinned_pos = _mesh.vertex_property<pmp::Point>(properties::vertexSkinnedPos);
    std::memcpy(skinned_pos.vector().data(), other_skinned_pos.vector().data(), n_vertices * sizeof(pmp::Point));

    // vertex skinned normal
    auto other_skinned_normal = other._mesh.get_vertex_property<pmp::Normal>(properties::vertexSkinnedNormal);
    auto skinned_normal = _mesh.vertex_property<pmp::Normal>(properties::vertexSkinnedNormal);
    std::memcpy(skinned_normal.vector().data(), other_skinned_normal.vector().data(), n_vertices * sizeof(pmp::Normal));

    // global pos
    auto other_global_pos = other._mesh.get_vertex_property<pmp::Point>(properties::vertexGlobalPosition);
    auto global_pos = _mesh.vertex_property<pmp::Point>(properties::vertexGlobalPosition);
    std::memcpy(global_pos.vector().data(), other_global_pos.vector().data(), n_vertices * sizeof(pmp::Point));

    // global normal
    auto other_global_normal = other._mesh.get_vertex_property<pmp::Normal>(properties::vertexGlobalNormal);
    auto global_normal = _mesh.vertex_property<pmp::Normal>(properties::vertexGlobalNormal);
    std::memcpy(global_normal.vector().data(), other_global_normal.vector().data(), n_vertices * sizeof(pmp::Normal));

    set_dirty(true);
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinnableMesh::skinning_setup(const Rig* rig) -> void
{
    if (rig == nullptr) {
        return;
    }

    // setup array of joint transformations, represented both as
    // 4x4 matrices and as 8D dual quaterions
    _skinning_matrices.resize(rig->joints_.size());
    _skinning_dualquats.resize(rig->joints_.size());

    for (unsigned int i = 0; i < rig->joints_.size(); ++i)
    {
        _skinning_matrices[i] = rig->joints_[i]->transformation_;
        _skinning_dualquats[i] = pmp_character::mat4_to_dualquat(rig->joints_[i]->transformation_);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinnableMesh::linear_skinning_matrices() -> std::vector<pmp::mat4>&
{
    return _skinning_matrices;
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinnableMesh::dualquat_skinning_matrices() -> std::vector<pmp_character::vec8>&
{
    return _skinning_dualquats;
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinnableMesh::transform() -> pmp::mat4&
{
    return _transform;
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinnableMesh::has_skinning_weights() const -> bool
{
    return _has_skinning_weights;
}

// ---------------------------------------------------------------------------------------------------------------------

auto SkinnableMesh::set_has_skinning_weights(const bool value) -> void
{
    _has_skinning_weights = value;
}

// ---------------------------------------------------------------------------------------------------------------------


// =====================================================================================================================

NAMESPACE_END(volumetric_base)

// =====================================================================================================================
