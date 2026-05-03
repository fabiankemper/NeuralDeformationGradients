//======================================================================================================================
//
// Created by Fabian Kemper on 2024-09-02.
//
//======================================================================================================================

#include "volumetric_state.h"

#include <ranges>

#include <volanim_utils/defs.h>
#include <volanim_utils/string_utils.h>

//======================================================================================================================

NAMESPACE_BEGIN(volumetric_base)

//======================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

VolumetricState::VolumetricState() = default;

// ---------------------------------------------------------------------------------------------------------------------

VolumetricState::~VolumetricState()
{
    delete highres_skeleton_warp;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::mean_surface_mesh(const Layer layer) -> const SurfaceMesh&
{
    return _mean_meshes[static_cast<std::underlying_type_t<Layer>>(layer)];
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::mean_mesh() -> const LayeredMesh&
{
    return _mean_mesh;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::set_mean_meshes(
    const SurfaceMesh& skel,
    const SurfaceMesh& musc,
    const SurfaceMesh& skin
) -> void
{
    _mean_meshes[static_cast<std::underlying_type_t<Layer>>(Layer::SKEL)] = skel;
    _mean_meshes[static_cast<std::underlying_type_t<Layer>>(Layer::MUSC)] = musc;
    _mean_meshes[static_cast<std::underlying_type_t<Layer>>(Layer::SKIN)] = skin;

    _mean_mesh = LayeredMesh(skel, musc, skin);
    update_revision();
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::set_layers(const LayeredMesh &mesh) -> void {
    _layered_mesh = mesh;

    update_revision();
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::get_layers() const -> const LayeredMesh & {
    return _layered_mesh;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::set_layers_from_mean() -> void
{
    set_layers(_mean_mesh);
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::set_template_layers(
    const SurfaceMesh& skel,
    const SurfaceMesh& musc,
    const SurfaceMesh& skin
) -> void
{
    _template_layers[static_cast<std::underlying_type_t<Layer>>(Layer::SKEL)] = skel;
    _template_layers[static_cast<std::underlying_type_t<Layer>>(Layer::MUSC)] = musc;
    _template_layers[static_cast<std::underlying_type_t<Layer>>(Layer::SKIN)] = skin;
    update_revision();
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::set_template_layers(const LayeredMesh& layers) -> void
{
    _template_layers[static_cast<std::underlying_type_t<Layer>>(Layer::SKEL)] = layers.to_mesh(Layer::SKEL);
    _template_layers[static_cast<std::underlying_type_t<Layer>>(Layer::MUSC)] = layers.to_mesh(Layer::MUSC);
    _template_layers[static_cast<std::underlying_type_t<Layer>>(Layer::SKIN)] = layers.to_mesh(Layer::SKIN);
    update_revision();
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::get_template_layer(const Layer layer) -> const SurfaceMesh&
{
    return _template_layers[static_cast<std::underlying_type_t<Layer>>(layer)];
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::get_template_mesh() -> LayeredMesh
{
    return LayeredMesh {
        get_template_layer(Layer::SKEL),
        get_template_layer(Layer::MUSC),
        get_template_layer(Layer::SKIN)
    };
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::get_template_layer_unsafe(Layer layer) -> SurfaceMesh&
{
    return _template_layers[static_cast<std::underlying_type_t<Layer>>(layer)];
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::set_highres_meshes(
    const SurfaceMesh& skel,
    const SurfaceMesh& musc,
    const SurfaceMesh& skin
) -> void
{
    set_highres_mesh(Layer::SKEL, skel);
    set_highres_mesh(Layer::MUSC, musc);
    set_highres_mesh(Layer::SKIN, skin);
    update_revision();
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::set_highres_mesh(Layer layer, const SurfaceMesh& mesh) -> void
{
    _highres_meshes[static_cast<std::underlying_type_t<Layer>>(layer)] = mesh;
    update_revision();
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::get_highres_mesh(const Layer layer) const -> const SurfaceMesh&
{
    return _highres_meshes[static_cast<std::underlying_type_t<Layer>>(layer)];
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::set_highres_wrap(Layer layer, const SurfaceMesh& mesh) -> void
{
    _highres_wraps[static_cast<std::underlying_type_t<Layer>>(layer)] = mesh;
    update_revision();
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::get_highres_wrap(Layer layer) -> SurfaceMesh&
{
    return _highres_wraps[static_cast<std::underlying_type_t<Layer>>(layer)];
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::set_skinning_parameters(
    const Layer layer,
    const SkinningParameters& skinning_parameters
) -> void
{
    _skinning_parameters[static_cast<std::underlying_type_t<Layer>>(layer)] = skinning_parameters;
    update_revision();
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::get_skinning_parameters(const Layer layer) const -> const SkinningParameters&
{
    return _skinning_parameters[static_cast<std::underlying_type_t<Layer>>(layer)];
}

// ---------------------------------------------------------------------------------------------------------------------

/**
 * Increase internal revision
 */
auto VolumetricState::update_revision() -> void
{
    ++revision;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::n_vertices() const -> size_t
{
    // return number of vertices of first mean mesh
    return _template_layers[0].n_vertices();
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::check_consistency() -> bool
{
    // mean meshes
    const auto n_vertices = _mean_meshes[0].n_vertices(); // _mean_meshes[0] defines vertex count
    if (_mean_meshes[1].n_vertices() != n_vertices) {
        std::cerr << "Mean_meshes[1].n_vertices() != " << n_vertices << '\n';
        return false;
    }
    if (_mean_meshes[2].n_vertices() != n_vertices) {
        std::cerr << "Mean_meshes[2].n_vertices() != " << n_vertices << '\n';
        return false;
    }

    // layered_mesh
    for (auto layer : ALL_LAYERS) {
        auto& layered_mesh = get_layers();
        if (layered_mesh.n_vertices() != n_vertices) {
            std::cerr << "layered_mesh[" << static_cast<std::underlying_type_t<Layer>>(layer) << "].n_vertices() != " << n_vertices << '\n';
            return false;
        }
    }

    // template wraps
    for (auto layer : ALL_LAYERS) {
        auto& wrap = get_template_layer(layer);
        if (wrap.n_vertices() != n_vertices) {
            std::cerr << "template_wrap[" << static_cast<std::underlying_type_t<Layer>>(layer) << "].n_vertices() != " << n_vertices << '\n';
            return false;
        }
    }

    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::is_initialized(const bool check_layers, bool check_highres) const -> bool
{
    // check mean mesh set
    if (_mean_mesh.n_vertices() == 0) {
        return false;
    }

    // check layers set
    if (check_layers) {
            if (get_layers().n_vertices()==0)
                return false;

    }

    // check template set
    if (std::ranges::any_of(ALL_LAYERS, [&](auto& layer) {
        return _template_layers[static_cast<std::underlying_type_t<Layer>>(layer)].n_vertices() == 0;
    })) {
        return false;
    }

    // check rig set
    if (rig.joints_.empty()) {
        return false;
    }

    // check skinning parameters set
    if (std::ranges::any_of(ALL_LAYERS, [&](auto& layer) {
        return _skinning_parameters[static_cast<std::underlying_type_t<Layer>>(layer)].empty();
    })) {
        return false;
    }

    // check high-res meshes
    if (check_highres) {
        if (std::ranges::any_of(ALL_LAYERS, [&](auto& layer) {
            return _highres_meshes[static_cast<std::underlying_type_t<Layer>>(layer)].n_vertices() == 0;
        })) {
            return false;
        }
    }

    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::options() -> nlohmann::json&
{
    return _options;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::get_scaled_highres_skeleton() -> const volumetric_base::SkinnableMesh&
{
    return _scaled_highres_skeleton;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::set_scaled_highres_skeleton(const volumetric_base::SkinnableMesh& mesh) -> void
{
    _scaled_highres_skeleton = mesh;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::get_bindpose() const -> std::vector<float>
{
    return _bindpose;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::set_bindpose(const std::vector<float>& bindpose) -> void
{
    _bindpose = bindpose;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::get_pose() const -> std::vector<float>
{
    return _pose;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::set_pose(const std::vector<float>& pose_rotvec) -> void
{
    _pose = pose_rotvec;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::get_lambda_weights() -> std::vector<float>&
{
    return _lambda_weights;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricState::get_skel_wrap_joint_spheres() -> std::vector<std::vector<std::tuple<int, pmp::vec3, float>>>&
{
    return _skel_wrap_joint_spheres;
}

// ---------------------------------------------------------------------------------------------------------------------

//======================================================================================================================

NAMESPACE_END(volumetric_base)

//======================================================================================================================
