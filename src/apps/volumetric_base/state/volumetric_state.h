// =====================================================================================================================
//
// Created by Fabian Kemper on 2024-09-02.
//
// =====================================================================================================================

#pragma once
#ifndef VOLUMETRIC_DATA_H
#define VOLUMETRIC_DATA_H

// =====================================================================================================================

#include "pmp_ext/defs.h"

#include <filesystem>
#include <memory_resource>
#include <unordered_set>

#include <pmp/surface_mesh.h>
#include <nlohmann/json.hpp>
#include <pmp_character/Types.h>
#include "volumetric_base/rbf_warp/rbf_warp_stephan.h"

#include "volumetric_base/aliases.h"
#include "volumetric_base/layered_mesh.h"
#include "volumetric_base/volumetric_types.h"
#include "volanim_utils/io/io_selection.h"
#include "volumetric_base/skinnable_mesh.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)

// =====================================================================================================================

using Path = std::filesystem::path;

using Selection = volanim::utils::Selection;
using SurfaceMesh = pmp::SurfaceMesh;
using Rig = pmp_character::Skeleton;

using ivec8 = pmp_character::ivec8; // joint indices for dependend joints
using vec8 = pmp_character::vec8; // weight for this dependency
using SkinningParameters = std::vector<std::tuple<ivec8, vec8>>;

// =====================================================================================================================

// forward declarations
namespace utils
{
    struct StateModification;
} // namespace utils
struct RetargetSkeleton;

// =====================================================================================================================

/**
* Data exchange object which contains all layers for the volumetric mesh
*/

class VolumetricState {

protected:
    std::array<SurfaceMesh, 3> _mean_meshes;
    volumetric_base::LayeredMesh _mean_mesh;

    // std::array<SurfaceMesh, LAYER_MESHES_CNT> _layers;
    volumetric_base::LayeredMesh _layered_mesh;


    bool dirty = false;
    // skinning parameters for each wrap (connectivity is given by mean meshes)
    std::array<SkinningParameters, 3> _skinning_parameters;

    // template wrap meshes (used for mean value coordinates calculation of rig)
    // also storing the skinning weights and skinning dependencies
    std::array<SurfaceMesh, 3> _template_layers;

    // Template high-res meshes
    std::array<SurfaceMesh, 3> _highres_meshes;
    // Wraps for high-res warping
    std::array<SurfaceMesh, 3> _highres_wraps;

    // arbitrary options
    nlohmann::json _options;

    // current rescaled high-res skeleton
    volumetric_base::SkinnableMesh _scaled_highres_skeleton;

    // bindpose (map a-pose to t-pose) as rotvecs (rodrigues axis angle)
    std::vector<float> _bindpose;

    // current pose as rotvecs
    std::vector<float> _pose;

    // TODO: current collision check groups, maybe not used
    std::vector<std::pmr::unordered_set<int>> _collision_groups;

    // specific lambda parameters / poisson ratios for vertices of character
    // 0 to 1, material parameter which is multiplied with the maximum lambda (volume preservation)
    // this allows center regions in knee, elbows and the stomach region to be more compressible
    // note: the vertex indices are in compressed format (collapsed vertices are not present)
    std::vector<float> _lambda_weights;

    // bone centers and spheres for each vertex of a joint
    // used for spherical skinning parameters
    // see Physically-Based Character Skinning by Crispin Deul and Jan Bender
    // vertex index ->
    //      for every joint ->
    //          [joint index, nearest point on joint center line, distance]
    std::vector<std::vector<std::tuple<int, pmp::vec3, float>>> _skel_wrap_joint_spheres;


public:
    VolumetricState();
    ~VolumetricState();

    // gender
    Gender gender = Gender::MALE;

    // rig (animation skeleton)
    Rig rig = Rig();
    LayeredMesh _skinning_result;

    // head, hands and toes
    Selection collapsed_vertices;

    // high-res skeleton warp
    volumetric_base::warping::RBFWarpStephan* highres_skeleton_warp = nullptr;

    // counting up if new files are loaded
    size_t revision = 0;

    auto set_skinning_result(const volumetric_base::LayeredMesh &mesh) -> void {
        _skinning_result = mesh;
        dirty = true;
    }

    auto set_dirty(const bool dirty) -> void  {
        this->dirty = dirty;
    }

    [[nodiscard]]
    auto get_dirty() const -> bool {
        return dirty;
    }

    // get mean mesh for given layer
    auto mean_surface_mesh(Layer layer) -> const SurfaceMesh&;
    auto mean_mesh() -> const volumetric_base::LayeredMesh&;

    // set mean meshes
    auto set_mean_meshes(const SurfaceMesh& skeleton, const SurfaceMesh& muscle, const SurfaceMesh& skin) -> void;

    /**
     * Set internal layers from given 3 persons
     * @param skeleton all three layers of the skeleton person (used for skeleton target)
     */
    auto set_layers(const LayeredMesh &skeleton) -> void;

    [[nodiscard]]
    auto get_layers() const -> const LayeredMesh &;

    // set all layers equal to mean
    auto set_layers_from_mean() -> void;

    /**
     * Set template wrap layers for calculation of mean value coordinates
     * Used for repositioning of the animation rig in newly loaded meshes
     *
     * @param skel skeleton wrap
     * @param musc muscle wrap
     * @param skin skin wrap
     */
    auto set_template_layers(const SurfaceMesh& skel, const SurfaceMesh& musc, const SurfaceMesh& skin) -> void;
    auto set_template_layers(const LayeredMesh& layers) -> void;
    auto get_template_layer(Layer layer) -> const SurfaceMesh&;
    auto get_template_mesh() -> LayeredMesh;

    /**
     * High resolution meshes from template
     * @param skel Skeleton (bones)
     * @param musc Muscles
     * @param skin Skin (identical to wraps)
     */
    auto set_highres_meshes(const SurfaceMesh& skel, const SurfaceMesh& musc, const SurfaceMesh& skin) -> void;
    auto set_highres_mesh(Layer layer, const SurfaceMesh& mesh) -> void;
    [[nodiscard]]
    auto get_highres_mesh(Layer layer) const -> const SurfaceMesh&;

    /**
     * High resolution mesh wraps
     */
    auto set_highres_wrap(Layer layer, const SurfaceMesh& mesh) -> void;
    auto get_highres_wrap(Layer layer) -> SurfaceMesh&;

    /**
     * Return skinning parameters as combination <depends,weight> for given layer
     * Note: the parameters must have a weight, dependency for each vertex in the mean mesh
     * @param layer layer
     * @param skinning_parameters vertex dependency on joints and vertex weight for those dependencies
     *                            in the respective layer of the mean mesh
     */
    auto set_skinning_parameters(Layer layer, const SkinningParameters& skinning_parameters) -> void;
    [[nodiscard]]
    auto get_skinning_parameters(Layer layer) const -> const SkinningParameters&;

    // Update internal revision
    auto update_revision() -> void;

    // Get number of vertices
    [[nodiscard]]
    auto n_vertices() const -> size_t;

    // check internal buffer consistency
    [[nodiscard]]
    auto check_consistency() -> bool;

    // check if initialized
    // true, if collapsed vertices, template, mean and layers are set
    [[nodiscard]]
    auto is_initialized(bool check_layers = true, bool check_highres = false) const -> bool;

    // get and set options
    [[nodiscard]]
    auto options() -> nlohmann::json&;

    // get current re-scaled high-res skeleton
    auto get_scaled_highres_skeleton() -> const volumetric_base::SkinnableMesh&;
    auto set_scaled_highres_skeleton(const volumetric_base::SkinnableMesh& mesh) -> void;

    // get bindpose
    [[nodiscard]]
    auto get_bindpose() const -> std::vector<float>;
    auto set_bindpose(const std::vector<float>& bindpose) -> void;

    // get and set current pose for rig
    [[nodiscard]]
    auto get_pose() const -> std::vector<float>;
    auto set_pose(const std::vector<float>& pose_rotvec) -> void;

    // get lambda weights for volume preservation (in poisson ratio)
    auto get_lambda_weights() -> std::vector<float>&;

    // get skel wrap joint sphere parameters
    auto get_skel_wrap_joint_spheres() -> std::vector<std::vector<std::tuple<int, pmp::vec3, float>>>&;

protected:
    // please do not use this method out of setup methods!
    auto get_template_layer_unsafe(Layer layer) -> SurfaceMesh&;

    // make friend classes for internal access
    friend struct utils::StateModification;
    friend struct RetargetSkeleton;
};

// =====================================================================================================================

NAMESPACE_END(volumetric_base)

// =====================================================================================================================

#endif //VOLUMETRIC_DATA_H

// =====================================================================================================================
