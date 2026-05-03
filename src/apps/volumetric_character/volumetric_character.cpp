// =====================================================================================================================

#include <utility>

#include <pmp/algorithms/utilities.h>
#include <pmp/mat_vec.h>
#include <pmp/io/io.h>

#include <npy.hpp>

#include "pmp_ext/pmp_matrix_ext.h"
#include "pmp_ext/pmp_io.h"

#include "volumetric_base/volumetric_types.h"
#include "volumetric_base/state/utils.h"
#include "volumetric_base/rbf_warp/rbf_warp.h"
#include "volumetric_base/utility/skinning_normalization.h"

#include "volumetric_base/state/retarget_skeleton/mean_value_coordinates.h"
#include "volumetric_character/volumetric_character.h"

#include "volanim_utils/matrix_utils.h"
#include "volumetric_character/skinning/dqs.h"
#include "volumetric_character/skinning/lbs.h"
#include "volumetric_character/skinning/tet_volumetric.h"

#include "../volumetric_skinning/base/basic_skinning.h"
#include "../volumetric_skinning/base/io.hpp"
#include "../volumetric_skinning/base/volumetric_boundary_fixer.h"
#include "volumetric_skinning/corotational_linear_elasticity/corotational_elastic_skinner.h"
#include "volumetric_skinning/neural_skinning/defgrad_skinner.h"
#include "volumetric_skinning/neural_skinning/vertex_skinner.h"
#include "volumetric_skinning/projective_skinning/projective_skinning.h"
#include "volumetric_skinning/stable_neo_hookean_elasticity/StableNeoHookeanSkinner.h"

// =====================================================================================================================

#define BENCHMARK_RBF_WARP 0

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_character)

// =====================================================================================================================

namespace properties = pmp_character::properties;

// =====================================================================================================================

using VolumetricFixer = volumetric_skinning::VolumetricBoundaryFixer;

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

VolumetricCharacter::VolumetricCharacter()
    :   _state(new VolumetricState()),
        _skinnable_meshes({
            SkinnableMesh(SurfaceMesh()),
            SkinnableMesh(SurfaceMesh()),
            SkinnableMesh(SurfaceMesh())
        }),
        _highres_meshes({
            SkinnableMesh(SurfaceMesh()),
            SkinnableMesh(SurfaceMesh()),
            SkinnableMesh(SurfaceMesh())
        })
{
    _skinner = new volumetric_skinning::LinearBlendSkinning();
}

// ---------------------------------------------------------------------------------------------------------------------

VolumetricCharacter::VolumetricCharacter(const VolumetricCharacter& other)
    :   _state(new VolumetricState(*other._state)),
        _skinnable_meshes({
            SkinnableMesh{other.skinnable_layer(Layer::SKEL)},
            SkinnableMesh{other.skinnable_layer(Layer::MUSC)},
            SkinnableMesh{other.skinnable_layer(Layer::SKIN)}
        }),
        _highres_meshes({
            SkinnableMesh{other.highres_layer(Layer::SKEL)},
            SkinnableMesh{other.highres_layer(Layer::MUSC)},
            SkinnableMesh{other.highres_layer(Layer::SKIN)}
        })
{
    delete _skinner;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::operator=(const VolumetricCharacter& other) -> VolumetricCharacter&
{
    // copy assignment constructor
    if (this != &other) {
        _state = other._state;
        skinnable_layer(Layer::SKEL) = other.skinnable_layer(Layer::SKEL);
        skinnable_layer(Layer::MUSC) = other.skinnable_layer(Layer::MUSC);
        skinnable_layer(Layer::SKIN) = other.skinnable_layer(Layer::SKIN);

        highres_layer(Layer::SKEL) = other.highres_layer(Layer::SKEL);
        highres_layer(Layer::MUSC) = other.highres_layer(Layer::MUSC);
        highres_layer(Layer::SKIN) = other.highres_layer(Layer::SKIN);
    }
    return *this;
}

// ---------------------------------------------------------------------------------------------------------------------

VolumetricCharacter::~VolumetricCharacter()
{
    delete _state;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::skinnable_layer(const Layer& layer) -> SkinnableMesh&
{
    return _skinnable_meshes[static_cast<std::underlying_type_t<Layer>>(layer)];
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::skinnable_layer(const Layer& layer) const -> const SkinnableMesh&
{
    return _skinnable_meshes[static_cast<std::underlying_type_t<Layer>>(layer)];
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::highres_layer(const Layer& layer) -> SkinnableMesh&
{
    return _highres_meshes[static_cast<std::underlying_type_t<Layer>>(layer)];
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::highres_layer(const Layer& layer) const -> const SkinnableMesh&
{
    return _highres_meshes[static_cast<std::underlying_type_t<Layer>>(layer)];
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::bounds() -> pmp::BoundingBox
{
    pmp::BoundingBox bounding_box;
    bounding_box += pmp::bounds(skinnable_layer(Layer::SKEL).mesh());
    bounding_box += pmp::bounds(skinnable_layer(Layer::MUSC).mesh());
    bounding_box += pmp::bounds(skinnable_layer(Layer::SKIN).mesh());
    return bounding_box;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::reinit_skinnable_layers() -> void
{
    // re-create skinnable meshes
    updated_template_layers();
    // set skinning parameters
    updated_skinning_parameters();
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::set_view_layers(
    const SurfaceMesh& skel_mesh,
    const SurfaceMesh& musc_mesh,
    const SurfaceMesh& skin_mesh
) -> void
{
    // tuples: target, source
    std::vector<std::tuple<SurfaceMesh&, const SurfaceMesh&> > zip_meshes{
        {skinnable_layer(Layer::SKEL).mesh(), skel_mesh},
        {skinnable_layer(Layer::MUSC).mesh(), musc_mesh},
        {skinnable_layer(Layer::SKIN).mesh(), skin_mesh}
    };

    // update vertex positions
    for (auto& [target, source]: zip_meshes) {
        auto skinned_positions = target.vertex_property<pmp::Point>(properties::vertexGlobalPosition);
        for (auto vertex: target.vertices()) {
            // copy vertex positions
            target.position(vertex) = source.position(vertex);
            // apply to animation positions
            skinned_positions[vertex] = source.position(vertex);
        }
    }
    // recalculate joint positions and reset new layers in bind pose
    recalculate_joints();
    // trigger update
    set_dirty();
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::updated_template_layers() -> void
{
    for (std::vector<Layer> layers{Layer::SKEL, Layer::MUSC, Layer::SKIN}; auto layer: layers) {
        auto& template_wrap = state()->get_template_layer(layer);
        // force setting the needed vertex properties
        skinnable_layer(layer).set_mesh(template_wrap);

        auto& highres_mesh = state()->get_highres_mesh(layer);
        highres_layer(layer).set_mesh(highres_mesh);
    }

    // update highres warper
    _hires_skinner.initialize(state());
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::updated_skinning_parameters() -> void
{
    // set vertices and connectivity
    for (std::vector<Layer> layers{Layer::SKEL, Layer::MUSC, Layer::SKIN}; auto layer: layers) {
        // get actual skinnable mesh (for properties)
        auto& mesh = skinnable_layer(layer).mesh();

        // set vdepends, vweights
        auto vdepend = mesh.vertex_property<volumetric_base::ivec8>(properties::vertexSkinningDepends);
        auto vweight = mesh.vertex_property<volumetric_base::vec8>(properties::vertexSkinningWeights);

        auto skinning_parameters = state()->get_skinning_parameters(layer);

        // set skinning parameters for skinnable mesh
        for (auto vertex : mesh.vertices()) {
            auto [vdepend_value, vweight_value] = skinning_parameters[vertex.idx()];
            vdepend[vertex] = vdepend_value;
            vweight[vertex] = vweight_value;
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::update_mvc() -> void
{
    // perform forward kinematics on rig
    // forward kinematics should not be performed, because we change the bindpose from A to T-pose
    // state()->rig.forward_kinematics();

    // use only skeleton wrap for MVC
    const SurfaceMesh& skel_wrap = state()->get_template_layer(Layer::SKEL);

    if (skel_wrap.n_vertices() == 0) {
        std::cerr << "[ERROR] update_set_mvc(): Template wrap wrapping is empty.\n";
        return;
    }

    if (not skel_wrap.has_vertex_property(
            volumetric_base::mvc::MVC_PROPERTY_NAME)) {
        throw std::runtime_error("VolumetricCharacter::update_mvc() - Template Wrap skeleton layer is missing MVC weights.");
    }

    // copy MVC from template skel wrap to skeleton wrap
    auto skel_wrap_mvc = skinnable_layer(Layer::SKEL).mesh().vertex_property<volumetric_base::mvc::MeanValueCoords>(
                volumetric_base::mvc::MVC_PROPERTY_NAME);
    auto template_skel_wrap_mvc = skel_wrap.get_vertex_property<volumetric_base::mvc::MeanValueCoords>(
            volumetric_base::mvc::MVC_PROPERTY_NAME);
    for (const auto vertex : skel_wrap.vertices()) {
        skel_wrap_mvc[vertex] = template_skel_wrap_mvc[vertex];
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::updated_mean_meshes() -> void
{
    // todo: THIS METHOD CAN BE REMOVED?
    // no - this method is a crucial part for mesh_combinator initialization

    if (state() == nullptr) {
        std::cerr << "[ERROR] Volumetric data not set.\n";
        return;
    }

    // create copies for skinning
    const SurfaceMesh& mean_skel = state()->mean_surface_mesh(Layer::SKEL);
    const SurfaceMesh& mean_musc = state()->mean_surface_mesh(Layer::MUSC);
    const SurfaceMesh& mean_skin = state()->mean_surface_mesh(Layer::SKIN);

    // dirty hack, set all layers from mean
    state()->set_layers_from_mean();

    // set shown layers to mean meshes
    set_view_layers(mean_skel, mean_musc, mean_skin);

    // update mean meshes in skinner
    if (_skinner != nullptr) {
        _skinner->updated_mean_layers(state());
    }

}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::set_translation(std::array<float, 3> translation) -> void
{
    // get root joint
    for (const auto& joint : state()->rig.joints_) {
        if (joint->name_ == "root") {
            joint->local_[12] = translation[0];
            joint->local_[13] = translation[1];
            joint->local_[14] = translation[2];
        }
    }

    // perform kinematics now
    state()->rig.forward_kinematics();
    set_skeleton_dirty(true); // trigger skinning
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::set_dirty(bool dirty) -> void
{
    skinnable_layer(Layer::SKEL).set_dirty(dirty);
    skinnable_layer(Layer::MUSC).set_dirty(dirty);
    skinnable_layer(Layer::SKIN).set_dirty(dirty);
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::set_skeleton_dirty(const bool dirty) -> void
{
    _skeleton_dirty.store(dirty);
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::set_highres_dirty(const bool dirty) -> void
{
    _highres_dirty.store(dirty);
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::skinning(const bool converge) -> void
{
    // measure time for solving skinning
    pmp::StopWatch watch;
    watch.start();

    // perform forward kinematics
    state()->rig.forward_kinematics();

    // check if initialized for methods, which do not rely on layers
    if (not _state->is_initialized(/*check_layers=*/false)) {
        std::cerr << "[ERROR] VolumetricCharacter::skinning - State is not initialized.\n";
        return;
    }

    // check for layered skinning methods
    if (
        _skinning_mode != SkinningMode::LINEAR_BLEND_SKINNING
        && _skinning_mode != SkinningMode::DUAL_QUATERNION_SKINNING
        && _skinning_mode != SkinningMode::VOLUMETRIC_SKINNING
        && _skinning_mode != SkinningMode::RBF_WARP
        && not _state->is_initialized(/*check_layers=*/true)
    ) {
        std::cerr << "[ERROR] Cannot perform layered skinning without layers loaded.\n";
        return;
    }

    if (_skinner != nullptr) {
        _skinner->perform_skinning(
            state(),
            skinnable_layer(Layer::SKEL),
            skinnable_layer(Layer::MUSC),
            skinnable_layer(Layer::SKIN),
            converge
        );
    } else {
        std::cout << "Illegal Skinning Mode " << static_cast<int>(_skinning_mode) << "\n" << std::endl;
        return;
    }
    watch.stop();
    current_fps = 0.9F * current_fps + 0.1F * (1.0F / watch.elapsed() * 1000.0F);
}


// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::set_new_skinner() -> void {

    // is the skinner already initialized and in cache?
    if (skinning_cache[_skinning_mode]) {
        std::cout << "Found already initialized skinner" << std::endl;
        _skinner = skinning_cache[_skinning_mode];
        _skinner->wake_up(); // signal reload after putting to cache
        return;
    }

    // create a new skinner based on the chosen skinning method
    switch (_skinning_mode) {
        case SkinningMode::LINEAR_BLEND_SKINNING:
            std::cout << "SkinningMode: LBS.\n";
            _skinner = new volumetric_skinning::LinearBlendSkinning();
            break;
        case SkinningMode::DUAL_QUATERNION_SKINNING:
            std::cout << "SkinningMode: DQS.\n";
            _skinner = new volumetric_skinning::DualQuaternionSkinning();
            break;
        case SkinningMode::VOLUMETRIC_SKINNING:
            std::cout << "SkinningMode: TetVolumetricSkinning.\n";
            _skinner = new volumetric_skinning::TetVolumetricSkinning();
            break;
        case SkinningMode::ELASTIC_SKINNING:
            std::cout << "SkinningMode: Elastic.\n";
            _skinner = new volumetric_skinning::CorotationalLinearElasticSkinner();
            break;
        case SkinningMode::PROJECTIVE_SKINNING:
            std::cout << "SkinningMode: Projective.\n";
            _skinner = new projective_skinning::ProjectiveSkinnerInterface();
            break;
        case SkinningMode::NN_DEFGRAD_SKINNING:
            std::cout << "SkinningMode: NN Skinning.\n";
            _skinner = new volumetric_skinning::network::DefgradNetworkSkinner();
            break;
        case SkinningMode::NN_VERTEX_SKINNING:
            std::cout << "SkinningMode: Vertex Net." << std::endl;
            _skinner = new volumetric_skinning::network::VertexNetworkSkinner();
            break;

        case SkinningMode::NEO_HOOKEAN:
            std::cout << "SkinningMode: Neo Hookean." << std::endl;
            _skinner = new volumetric_skinning::neo_hookean::NeoHookeanSkinner();
            break;
        default:
            _skinner = nullptr;
            break;
    }

    // store the new created skinner in the cache
    if (_skinner) {
        skinning_cache[_skinning_mode] = _skinner;
    }
}

// ---------------------------------------------------------------------------------------------------------------------


auto VolumetricCharacter::set_skinning_mode(const SkinningMode skinning_mode) -> void
{
    if (skinning_mode == _skinning_mode) {
        // nothing changed
        return;
    }

    _skinning_mode = skinning_mode;
    // std::cout << "Skinning mode: " << std::to_string(static_cast<std::underlying_type_t<Layer>>(skinning_mode)) << std::endl;
    set_new_skinner();

    // re-create skinnable layers
    reinit_skinnable_layers();
    // initialize everything
    if (_skinner != nullptr) {
        // set mean meshes
        _skinner->updated_mean_layers(state());
        // layers set? update for skinning
        if (_state->is_initialized(true)) {
            _skinner->updated_input_layers(state());
        }
        // perform pose
        _skinner->updated_pose(state());
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::set_pose(const std::vector<float>& joint_angles) -> void
{
    volumetric_base::utils::StateModification::set_pose(state(), joint_angles);

    if (_skinner != nullptr) {
        _skinner->updated_pose(state());
    }

    // re-render skeleton
    set_skeleton_dirty(true);
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::set_state(VolumetricState* state) -> void
{
    _state = state;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::state() const -> VolumetricState*
{
    return _state;
}


// ---------------------------------------------------------------------------------------------------------------------

void VolumetricCharacter::update_animators()
{
    const volumetric_base::LayeredMesh skeleton = _state->get_layers();
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::update_layers() -> void
{
    // recalculate layers in LBS and DQS
    _layers_dirty.store(true);

    // re-create skinnable layers, if vertices mismatch
    if (skinnable_layer(Layer::SKEL).mesh().n_vertices() != state()->get_layers().n_vertices()) {
        skinnable_layer(Layer::SKEL) = SkinnableMesh(state()->get_layers());
        std::cerr << "[WARN] Updated skinnable mesh due vertex mismatch. "
                     "This might occur due to incomplete initialization of the volumetric character. "
                     "Call updated_mean_meshes() and reinit_skinnable_layers() after setup!\n";
    }
    if (skinnable_layer(Layer::MUSC).mesh().n_vertices() != state()->get_layers().n_vertices()) {
        skinnable_layer(Layer::MUSC) = SkinnableMesh(state()->get_layers());
    }
    if (skinnable_layer(Layer::SKIN).mesh().n_vertices() != state()->get_layers().n_vertices()) {
        skinnable_layer(Layer::SKIN) = SkinnableMesh(state()->get_layers());
    }

    // replace head in skeleton
    SurfaceMesh skeleton = _state->get_layers().to_mesh(Layer::SKEL);
    skinnable_layer(Layer::SKEL).positions_from_baked_mesh(skeleton);

    // update head hands and toes for muscles and skin layer
    // todo: @fabian Nullpointer check
    for (auto v_idx: _state->collapsed_vertices) {
        const auto vertex = pmp::Vertex(v_idx);
        skinnable_layer(Layer::MUSC).mesh().position(vertex) = skeleton.position(vertex);
        skinnable_layer(Layer::SKIN).mesh().position(vertex) = skeleton.position(vertex);
    }

    // update animation skeleton
    recalculate_joints();

    if (use_template_mesh) {
        const auto template_mesh = state()->get_template_mesh();
        state()->set_layers(template_mesh);
    }
    update_animators();

    // update layers for skinner
    if (_skinner != nullptr) {
        _skinner->updated_input_layers(state());
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::imgui_skinner() const -> void
{
    if (_skinner != nullptr) {
        _skinner->imgui_update(state());
    }
}

//----------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::update_use_mean_mesh() -> void {
    update_layers();
    this->skinning();
}

// ---------------------------------------------------------------------------------------------------------------------
#if 0
auto VolumetricCharacter::calculate_high_res_anatomy_rbf(const long n_rbf_kernels) -> void
{
    std::cout << "Calculating new high-res anatomy\n";
    // check pre-conditions
    if (_skinnable_meshes[0].mesh().n_vertices() != state()->get_template_layer(Layer::SKEL).n_vertices()) {
        throw std::runtime_error("Skinnable meshes must have the same number of vertices as template highres skeleton.");
    }
    if (_skinnable_meshes[1].mesh().n_vertices() != state()->get_template_layer(Layer::MUSC).n_vertices()) {
        throw std::runtime_error("Skinnable meshes must have the same number of vertices as template highres muscles.");
    }

    // create new rbf warp using centers from all 3 layers
    const auto skel_wrap_tmpl = state()->get_template_layer(Layer::SKEL);
    const auto musc_wrap_tmpl = state()->get_template_layer(Layer::MUSC);
    const auto skin_tmpl = state()->get_template_layer(Layer::SKIN);
    const auto n_verts = static_cast<Eigen::Index> (skin_tmpl.n_vertices());

    // stack all control positions (undeformed)
    Eigen::Matrix3Xf undeformed_control = Eigen::Matrix3Xf::Zero(3, n_verts * 3);
    undeformed_control.block(0, 0 * n_verts, 3, n_verts) = volanim::utils::mesh_pos_to_matrix(skel_wrap_tmpl);
    undeformed_control.block(0, 1 * n_verts, 3, n_verts) = volanim::utils::mesh_pos_to_matrix(musc_wrap_tmpl);
    undeformed_control.block(0, 2 * n_verts, 3, n_verts) = volanim::utils::mesh_pos_to_matrix(skin_tmpl);

    // // debug: write undeformed
    // pmp::write(skel_wrap_tmpl, "out/undeformed_skel.obj");
    // pmp::write(musc_wrap_tmpl, "out/undeformed_musc.obj");
    // pmp::write(skin_tmpl, "out/undeformed_skin.obj");

    // init warp
    warping::RBFWarp template_warp { warping::RBFWarp::Poly::Quadratic };
    template_warp.init_warp(undeformed_control.cast<double>(), n_rbf_kernels);

    // get deformed wraps
    const auto deformed_wr_skel = _skinnable_meshes[0].bake_mesh();
    const auto deformed_wr_musc = _skinnable_meshes[1].bake_mesh();
    const auto deformed_wr_skin = _skinnable_meshes[2].bake_mesh();

    // stack deformed control positions
    Eigen::Matrix3Xf deformed_control = Eigen::Matrix3Xf::Zero(3, n_verts * 3);
    deformed_control.block(0, 0 * n_verts, 3, n_verts) = volanim::utils::mesh_pos_to_matrix(deformed_wr_skel);
    deformed_control.block(0, 1 * n_verts, 3, n_verts) = volanim::utils::mesh_pos_to_matrix(deformed_wr_musc);
    deformed_control.block(0, 2 * n_verts, 3, n_verts) = volanim::utils::mesh_pos_to_matrix(deformed_wr_skin);

    // // debug: write deformed
    // pmp::write(deformed_wr_skel, "out/deformed_skel.obj");
    // pmp::write(deformed_wr_musc, "out/deformed_musc.obj");
    // pmp::write(deformed_wr_skin, "out/deformed_skin.obj");

    // prepare highres meshes
    auto hires_skel = state()->get_highres_mesh(Layer::SKEL);
    auto hires_musc = state()->get_highres_mesh(Layer::MUSC);
    const auto hires_skel_nverts = static_cast<Eigen::Index> (hires_skel.n_vertices());
    const auto hires_musc_nverts = static_cast<Eigen::Index> (hires_musc.n_vertices());

    // stack high resolution meshes
    Eigen::Matrix3Xf hires_stacked = Eigen::Matrix3Xf::Zero(3, (hires_skel_nverts + hires_musc_nverts));
    hires_stacked.block(0, 0, 3, hires_skel_nverts) = volanim::utils::mesh_pos_to_matrix(hires_skel);
    hires_stacked.block(0, hires_skel_nverts, 3, hires_musc_nverts) = volanim::utils::mesh_pos_to_matrix(hires_musc);

    // perform warp
    Eigen::Matrix3Xf hires_warped = template_warp.apply_warp(deformed_control.cast<double>(), hires_stacked.cast<double>()).cast<float>();

    // store results
    volanim::utils::mesh_pos_from_matrix(hires_skel, hires_warped.block(0, 0, 3, hires_skel_nverts));
    volanim::utils::mesh_pos_from_matrix(hires_musc, hires_warped.block(0, hires_skel_nverts, 3, hires_musc_nverts));

    // write results to disk
    const auto skin_mesh = _skinnable_meshes[2].bake_mesh();
    pmp::write(hires_skel, "out/rbf_warped_highres_skel.obj");
    pmp::write(hires_musc, "out/rbf_warped_highres_musc.obj");
    pmp::write(skin_mesh, "out/rbf_skin_mesh.obj");

    // now there are high-res meshes available
    set_highres_dirty(true);

    std::cout << "Wrote RBF Warped high-res anatomy (out/...)." << std::endl;
}
#endif

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::calculate_high_res_anatomy_rbf(const long n_rbf_kernels) -> void
{
    std::cout << "Calculating new high-res anatomy\n";
    // check pre-conditions
    if (_skinnable_meshes[0].mesh().n_vertices() != state()->get_template_layer(Layer::SKEL).n_vertices()) {
        throw std::runtime_error("Skinnable meshes must have the same number of vertices as template highres skeleton.");
    }
    if (_skinnable_meshes[1].mesh().n_vertices() != state()->get_template_layer(Layer::MUSC).n_vertices()) {
        throw std::runtime_error("Skinnable meshes must have the same number of vertices as template highres muscles.");
    }

    // create new rbf warp using centers from all 3 layers
    const auto skel_wrap_tmpl = state()->get_template_layer(Layer::SKEL);
    const auto musc_wrap_tmpl = state()->get_template_layer(Layer::MUSC);
    const auto skin_tmpl = state()->get_template_layer(Layer::SKIN);

    // stack all control positions (undeformed)
    const Eigen::Matrix3Xf skel_undeformed = volanim::utils::mesh_pos_to_matrix(skel_wrap_tmpl);
    const Eigen::Matrix3Xf musc_undeformed = volanim::utils::mesh_pos_to_matrix(musc_wrap_tmpl);


    // // debug: write undeformed
    // pmp::write(skel_wrap_tmpl, "out/undeformed_skel.obj");
    // pmp::write(musc_wrap_tmpl, "out/undeformed_musc.obj");
    // pmp::write(skin_tmpl, "out/undeformed_skin.obj");

    // init warp
    volumetric_base::warping::RBFWarp skel_warp {
        volumetric_base::warping::RBFWarp::Poly::Quadratic };
    skel_warp.init_warp(skel_undeformed.cast<double>(), n_rbf_kernels);

    volumetric_base::warping::RBFWarp musc_warp {
        volumetric_base::warping::RBFWarp::Poly::Quadratic };
    musc_warp.init_warp(musc_undeformed.cast<double>(), n_rbf_kernels);

#if BENCHMARK_RBF_WARP
for (int i = 0; i < 100; i++) {
    auto start = std::chrono::high_resolution_clock::now();
#endif

    // get deformed wraps
    const auto deformed_wr_skel = _skinnable_meshes[0].bake_mesh();
    const auto deformed_wr_musc = _skinnable_meshes[1].bake_mesh();
    const auto deformed_wr_skin = _skinnable_meshes[2].bake_mesh();

    // stack deformed control positions
    Eigen::Matrix3Xf skel_deformed = volanim::utils::mesh_pos_to_matrix(deformed_wr_skel);
    Eigen::Matrix3Xf musc_deformed = volanim::utils::mesh_pos_to_matrix(deformed_wr_musc);

    // // debug: write deformed
    // pmp::write(deformed_wr_skel, "out/deformed_skel.obj");
    // pmp::write(deformed_wr_musc, "out/deformed_musc.obj");
    // pmp::write(deformed_wr_skin, "out/deformed_skin.obj");

    // prepare highres meshes
    auto hires_skel = state()->get_highres_mesh(Layer::SKEL);
    auto hires_musc = state()->get_highres_mesh(Layer::MUSC);

    // high-res undeformed
    const Eigen::Matrix3Xf hires_skel_pts = volanim::utils::mesh_pos_to_matrix(hires_skel);
    const Eigen::Matrix3Xf hires_musc_pts = volanim::utils::mesh_pos_to_matrix(hires_musc);

    // perform warp
    const Eigen::Matrix3Xf hires_skel_warped_pts = skel_warp.apply_warp(skel_deformed.cast<double>(), hires_skel_pts.cast<double>()).cast<float>();
    const Eigen::Matrix3Xf hires_musc_warped_pts = musc_warp.apply_warp(musc_deformed.cast<double>(), hires_musc_pts.cast<double>()).cast<float>();

    // store results
    volanim::utils::mesh_pos_from_matrix(hires_skel, hires_skel_warped_pts);
    volanim::utils::mesh_pos_from_matrix(hires_musc, hires_musc_warped_pts);

#if BENCHMARK_RBF_WARP
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "RBF Warp: " << (static_cast<float>(duration.count()) / 1000.0) << " ms" << std::endl;
}
#endif

#if !BENCHMARK_RBF_WARP
    // write results to disk
    const auto skin_mesh = _skinnable_meshes[2].bake_mesh();
    pmp::write(hires_skel, "out/rbf_warped_highres_skel.obj");
    pmp::write(hires_musc, "out/rbf_warped_highres_musc.obj");
    pmp::write(skin_mesh, "out/rbf_skin_mesh.obj");
#endif

    // now there are high-res meshes available
    set_highres_dirty(true);

    std::cout << "Wrote RBF Warped high-res anatomy (out/...)." << std::endl;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::save_meshes(const std::filesystem::path& out_dir, const std::string& prefix, VolIOFlags flags) -> void
{
    // create output directory
    if (not exists(out_dir)) {
        std::filesystem::create_directory(out_dir);
    }

    const bool _hires_before = highres_skinning;

    // calculate highres meshes if not already done
    if (not _hires_before) {
        highres_skinning = true;
        skinning();
    }

    // get all meshes
    const auto& skel_mesh = skinnable_layer(volumetric_character::Layer::SKEL).bake_mesh();
    const auto& musc_mesh = skinnable_layer(volumetric_character::Layer::MUSC).bake_mesh();
    const auto& skin_mesh = skinnable_layer(volumetric_character::Layer::SKIN).bake_mesh();

    volumetric_base::LayeredMesh wrap_layers{skel_mesh, musc_mesh, skin_mesh};
    pmp::SurfaceMesh musc_mesh_hires= _hires_skinner._barycentric_warper->get_warped_muscle(wrap_layers);
    pmp::SurfaceMesh skel_mesh_hires= _hires_skinner._barycentric_warper->get_warped_skeleton(wrap_layers);


    // w = wrap
    const auto skel_h_filename = out_dir / (prefix + "skel_hires.obj");
    const auto musc_h_filename = out_dir / (prefix + "musc_hires.obj");
    const auto skel_w_filename = out_dir / (prefix + "skel_wrap.obj");
    const auto musc_w_filename = out_dir / (prefix + "musc_wrap.obj");
    const auto skin_filename = out_dir / (prefix + "skin.obj");

    // hires layers
    if (flags.skin) {
        pmp_ext::write_small_obj(skin_mesh, skin_filename);
    }
    if (flags.hires_musc) {
        pmp_ext::write_small_obj(musc_mesh_hires, musc_h_filename);
    }
    if (flags.hires_skel) {
        pmp_ext::write_small_obj(skel_mesh_hires, skel_h_filename);
    }

    // wraps
    if (flags.musc_wrap) {
        pmp_ext::write_small_obj(musc_mesh, musc_w_filename);
    }
    if (flags.skel_wrap) {
        pmp_ext::write_small_obj(skel_mesh, skel_w_filename);
    }

    // reset
    highres_skinning = _hires_before;
}

// ---------------------------------------------------------------------------------------------------------------------

auto VolumetricCharacter::recalculate_joints() const -> void
{
    volumetric_base::utils::StateModification::retarget_skeleton(state());
}

// ---------------------------------------------------------------------------------------------------------------------


// =====================================================================================================================

NAMESPACE_END(volumetric_character)

// =====================================================================================================================
