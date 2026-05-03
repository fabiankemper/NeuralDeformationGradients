// =====================================================================================================================
//
// Created by fabian on 16/04/25.
//
// =====================================================================================================================

#include "skinning.h"

#include "pmp_character/DualQuaternion.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(skinning)

// =====================================================================================================================

namespace Properties = pmp_character::properties;

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

auto lbs(const VolumetricState* state, SkinnableMesh& skinnable_mesh, const Layer fallback_weights_layer) -> void
{
    if (state == nullptr) {
        throw std::runtime_error("LBS: state is nullptr.");
    }

    if (skinnable_mesh.has_skinning_weights()) {
        // mesh has own skinning weights, use them
        lbs_skinning(skinnable_mesh, &state->rig);
    } else {
        const auto& skinning_weights = state->get_skinning_parameters(fallback_weights_layer);
        // the mesh does not have own skinning weights, use the weights from state
        lbs_skinning(skinnable_mesh, &state->rig, skinning_weights);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto dqs(const VolumetricState* state, SkinnableMesh& skinnable_mesh, const Layer fallback_weights_layer) -> void
{
    if (state == nullptr) {
        throw std::runtime_error("LBS: state is nullptr.");
    }

    if (skinnable_mesh.has_skinning_weights()) {
        // mesh has own skinning weights, use them
        dqs_skinning(skinnable_mesh, &state->rig);
    } else {
        // the mesh does not have own skinning weights, use the weights from state
        const auto& skinning_weights = state->get_skinning_parameters(fallback_weights_layer);
        dqs_skinning(skinnable_mesh, &state->rig, skinning_weights);
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto add_skinning_weights(SkinnableMesh& skinnable_mesh, const SkinningParameters& parameters) -> void
{
    auto& mesh = skinnable_mesh.mesh();

    if (parameters.size() != mesh.n_vertices()) {
        throw std::runtime_error("lbs_skinning: skinning_parameter count does not match vertex count.");
    }

    // add properties
    auto v_depends = mesh.vertex_property<pmp_character::ivec8>(Properties::vertexSkinningDepends);
    auto v_weights = mesh.vertex_property<pmp_character::vec8>(Properties::vertexSkinningWeights);

    pmp::IndexType index = 0;
    for (const auto& [depends, weights] : parameters) {
        v_depends[pmp::Vertex(index)] = depends;
        v_weights[pmp::Vertex(index)] = weights;
        index++;
    }

    // mark this mesh as it now has usable skinning weights
    skinnable_mesh.set_has_skinning_weights(true);
}

// ---------------------------------------------------------------------------------------------------------------------

auto lbs_skinning(SkinnableMesh& skinnable_mesh, const volumetric_base::Rig* rig) -> void
{
    skinnable_mesh.skinning_setup(rig);
    auto& mesh = skinnable_mesh.mesh();

    auto skinned_vertices_ = mesh.vertex_property<pmp::Point>(Properties::vertexSkinnedPos);
    auto skinned_normals_ = mesh.vertex_property<pmp::Normal>(Properties::vertexSkinnedNormal);
    auto global_pos = mesh.vertex_property<pmp::Point>(Properties::vertexGlobalPosition);
    auto global_normal = mesh.vertex_property<pmp::Normal>(Properties::vertexGlobalNormal);

    // resize skinned vertex and normal vectors
    auto normals = mesh.vertex_property<pmp::Normal>(Properties::vertexNormals);

    // skinning parameters
    auto v_depends = mesh.vertex_property<pmp_character::ivec8>(Properties::vertexSkinningDepends);
    auto v_weights = mesh.vertex_property<pmp_character::vec8>(Properties::vertexSkinningWeights);

    const auto& skinning_matrices = skinnable_mesh.linear_skinning_matrices();
    const auto& transform = skinnable_mesh.transform();

    // OMP_PARALLEL_FOR()
    for (const auto v : mesh.vertices())
    {
        pmp::Vector<float, 3> currentVertex = mesh.position(v);
        pmp::Vector<float, 3> currentNormal = normals[v];
        auto blend_matrix = pmp::mat4(0);
        for (int j = 0; j < 8; j++)
        {
            blend_matrix += v_weights[v][j] * skinning_matrices[v_depends[v][j]];
        }

        // update final positions
        skinned_vertices_[v] = affine_transform(blend_matrix, currentVertex);
        skinned_normals_[v] =  linear_transform(blend_matrix, currentNormal);
        global_pos[v] = pmp::affine_transform(transform, skinned_vertices_[v]);
        global_normal[v] = pmp::affine_transform(transform,skinned_normals_[v]);
    }

    skinnable_mesh.set_dirty(true);
}

// ---------------------------------------------------------------------------------------------------------------------

auto dqs_skinning(SkinnableMesh& skinnable_mesh, const volumetric_base::Rig* rig) -> void
{
    skinnable_mesh.skinning_setup(rig);
    auto& mesh = skinnable_mesh.mesh();

    auto skinned_vertices_ = mesh.get_vertex_property<pmp::Point>(Properties::vertexSkinnedPos);
    auto skinned_normals_ = mesh.get_vertex_property<pmp::Normal>(Properties::vertexSkinnedNormal);
    auto global_pos = mesh.get_vertex_property<pmp::Point>(Properties::vertexGlobalPosition);
    auto global_normal = mesh.get_vertex_property<pmp::Normal>(Properties::vertexGlobalNormal);

    // resize skinned vertex and normal vectors
    auto normals = mesh.get_vertex_property<pmp::Normal>(Properties::vertexNormals);

    // skinning weights and dependencies
    // auto v_depends = _mesh.get_vertex_property<ivec8>(properties::vertexSkinningDepends);
    // auto v_weights = _mesh.get_vertex_property<vec8>(properties::vertexSkinningWeights);
    // auto blend_normal = child.mesh.get_vertex_property<pmp::vec3>(properties::vertexBlendNormal);

    pmp::VertexProperty<pmp::vec3> blendpos;
    if (mesh.has_vertex_property(Properties::vertexBlendPosition))
    {
        blendpos = mesh.get_vertex_property<pmp::vec3>(Properties::vertexBlendPosition);
    }
    else
    {
        blendpos = mesh.get_vertex_property<pmp::vec3>(Properties::vertexPosition);
    }

    const auto& skinning_dualquats = skinnable_mesh.dualquat_skinning_matrices();
    const auto& transform = skinnable_mesh.transform();

    // skinning parameters
    auto v_depends = mesh.vertex_property<pmp_character::ivec8>(Properties::vertexSkinningDepends);
    auto v_weights = mesh.vertex_property<pmp_character::vec8>(Properties::vertexSkinningWeights);

    for (auto v : mesh.vertices())
    {
        vec8 quatsum = vec8(0);
        // q1 und q2 sind die aktuelle und die neue skinnindualquat in der forschleife
        for (int j = 0; j < 8; j++)
        {
            vec8 temp = skinning_dualquats[v_depends[v][j]];
            quatsum += (pmp::dot(pmp_character::get_first_quat(quatsum), pmp_character::get_first_quat(temp)) > 0) ? temp * v_weights[v][j]
                                                                                  : -temp * v_weights[v][j];
            // quatsum += v_weights_[i][j]  * skinning_dualquats_[v_depends_[i][j]];
        }
        // Normalsieren
        quatsum /= pmp::norm(pmp_character::get_first_quat(quatsum));

        pmp::mat4 transmatrix = pmp_character::dualquat_to_mat4(quatsum);

        // update final positions
        skinned_vertices_[v] = affine_transform(transmatrix, blendpos[v]);
        skinned_normals_[v] = linear_transform(transmatrix, normals[v]);
        global_pos[v] = pmp::affine_transform(transform, skinned_vertices_[v]);
        global_normal[v] = pmp::affine_transform(transform,skinned_normals_[v]);
    }

    skinnable_mesh.set_dirty(true);
}

auto lbs_skinning(
    SkinnableMesh& skinnable_mesh,
    const volumetric_base::Rig* rig,
    const SkinningParameters& skinning_parameters
) -> void
{
    // set-up memory for skinning matrices
    skinnable_mesh.skinning_setup(rig);
    auto& mesh = skinnable_mesh.mesh();

    if (skinning_parameters.size() != mesh.n_vertices()) {
        throw std::runtime_error("lbs_skinning: skinning_parameter count does not match vertex count.");
    }

    auto skinned_vertices_ = mesh.get_vertex_property<pmp::Point>(Properties::vertexSkinnedPos);
    auto skinned_normals_ = mesh.get_vertex_property<pmp::Normal>(Properties::vertexSkinnedNormal);
    auto global_pos = mesh.get_vertex_property<pmp::Point>(Properties::vertexGlobalPosition);
    auto global_normal = mesh.get_vertex_property<pmp::Normal>(Properties::vertexGlobalNormal);

    // resize skinned vertex and normal vectors
    auto normals = mesh.get_vertex_property<pmp::Normal>(Properties::vertexNormals);
    const auto transform = skinnable_mesh.transform();
    const auto& skinning_matrices = skinnable_mesh.linear_skinning_matrices();

    // blendshapes (not supported)
    // pmp::VertexProperty<pmp::vec3> blendpos;
    // if (_mesh.has_vertex_property(properties::vertexBlendPosition))
    // {
    //     blendpos = _mesh.get_vertex_property<pmp::vec3>(properties::vertexBlendPosition);
    // }
    // else {
    //     blendpos = _mesh.get_vertex_property<pmp::vec3>(properties::vertexPosition);
    // }

    // skinning weights and dependencies
    // auto v_depends = _mesh.get_vertex_property<ivec8>(properties::vertexSkinningDepends);
    // auto v_weights = _mesh.get_vertex_property<vec8>(properties::vertexSkinningWeights);
    // auto blend_normal = child.mesh.get_vertex_property<pmp::vec3>(properties::vertexBlendNormal);

    // OMP_PARALLEL_FOR()
    for (auto v : mesh.vertices())
    {
        const auto& [v_depends, v_weights] = skinning_parameters[v.idx()];
        pmp::Vector<float, 3> currentVertex = mesh.position(v);
        pmp::Vector<float, 3> currentNormal = normals[v];
        auto blend_matrix = pmp::mat4(0);
        for (int j = 0; j < 8; j++)
        {
            blend_matrix += v_weights[j] * skinning_matrices[v_depends[j]];
        }

        // update final positions
        skinned_vertices_[v] = affine_transform(blend_matrix, currentVertex);
        skinned_normals_[v] =  linear_transform(blend_matrix, currentNormal);
        global_pos[v] = pmp::affine_transform(transform, skinned_vertices_[v]);
        global_normal[v] = pmp::affine_transform(transform, skinned_normals_[v]);
    }

    skinnable_mesh.set_dirty(true);
}

// ---------------------------------------------------------------------------------------------------------------------

auto dqs_skinning(
    SkinnableMesh& skinnable_mesh,
    const volumetric_base::Rig* rig,
    const SkinningParameters& skinning_parameters
) -> void
{
    // set-up memory for skinning matrices
    skinnable_mesh.skinning_setup(rig);
    const auto& mesh = skinnable_mesh.mesh();

    if (skinning_parameters.size() != mesh.n_vertices()) {
        throw std::runtime_error("lbs_skinning: skinning_parameter count does not match vertex count.");
    }

    auto skinned_vertices_ = mesh.get_vertex_property<pmp::Point>(Properties::vertexSkinnedPos);
    auto skinned_normals_ = mesh.get_vertex_property<pmp::Normal>(Properties::vertexSkinnedNormal);
    auto global_pos = mesh.get_vertex_property<pmp::Point>(Properties::vertexGlobalPosition);
    auto global_normal = mesh.get_vertex_property<pmp::Normal>(Properties::vertexGlobalNormal);

    // resize skinned vertex and normal vectors
    auto normals = mesh.get_vertex_property<pmp::Normal>(Properties::vertexNormals);

    // skinning weights and dependencies
    // auto v_depends = _mesh.get_vertex_property<ivec8>(Properties::vertexSkinningDepends);
    // auto v_weights = _mesh.get_vertex_property<vec8>(Properties::vertexSkinningWeights);
    // auto blend_normal = child.mesh.get_vertex_property<pmp::vec3>(Properties::vertexBlendNormal);

    pmp::VertexProperty<pmp::vec3> blendpos;
    if (mesh.has_vertex_property(Properties::vertexBlendPosition))
    {
        blendpos = mesh.get_vertex_property<pmp::vec3>(Properties::vertexBlendPosition);
    }
    else
    {
        blendpos = mesh.get_vertex_property<pmp::vec3>(Properties::vertexPosition);
    }

    const auto transform = skinnable_mesh.transform();
    const auto& skinning_dualquats = skinnable_mesh.dualquat_skinning_matrices();


    for (auto v : mesh.vertices())
    {
        const auto& [v_depends, v_weights] = skinning_parameters[v.idx()];
        vec8 quatsum = vec8(0);
        // q1 und q2 sind die aktuelle und die neue skinnindualquat in der forschleife
        for (int j = 0; j < 8; j++)
        {
            vec8 temp = skinning_dualquats[v_depends[j]];
            quatsum += (pmp::dot(pmp_character::get_first_quat(quatsum), pmp_character::get_first_quat(temp)) > 0) ? temp * v_weights[j]
                                                                                  : -temp * v_weights[j];
            // quatsum += v_weights_[i][j]  * skinning_dualquats_[v_depends_[i][j]];
        }
        // Normalsieren
        quatsum /= pmp::norm(pmp_character::get_first_quat(quatsum));

        pmp::mat4 transmatrix = pmp_character::dualquat_to_mat4(quatsum);

        // update final positions
        skinned_vertices_[v] = affine_transform(transmatrix, blendpos[v]);
        skinned_normals_[v] = linear_transform(transmatrix, normals[v]);
        global_pos[v] = pmp::affine_transform(transform, skinned_vertices_[v]);
        global_normal[v] = pmp::affine_transform(transform, skinned_normals_[v]);
    }

    skinnable_mesh.set_dirty(true);
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(skinning)
NAMESPACE_END(volumetric_base)

// =====================================================================================================================
