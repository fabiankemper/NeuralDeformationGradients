// =====================================================================================================================
// Created by fabian on 3/28/25.
// mail@fabiankemper.de
// =====================================================================================================================

#include "retarget_skeleton.h"

#include "mean_value_coordinates.h"
#include "volanim_utils/compiler_support.h"
#include "volumetric_base/state/utils.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)

// =====================================================================================================================

using Line = std::tuple<pmp::vec3, pmp::vec3>; // center, direction r(t) = c + d * t
// joint_idx, point on joint center line in initial pose, distance from vertex to center line
using SphereParams = std::tuple<int, pmp::vec3, float>;

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

auto get_joint_center_lines(VolumetricState* state) -> std::vector<Line>
{
    std::vector<Line> joint_center_lines;
    const auto& rig = state->rig;
    // ensure all global matrices are up to date
    rig.forward_kinematics();

    for (const auto& joint : rig.joints_) {
        // center point is the global translation
        pmp::vec3 center = joint->get_global_initial_translation();
        pmp::vec3 direction;

        const auto& children = joint->children_;

        if (not children.empty()) {
            // average all children positions
            pmp::vec3 mean_pos = pmp::vec3(0.0f);
            for (auto* child : children) {
                mean_pos += child->get_global_initial_translation();
            }
            mean_pos /= static_cast<pmp::Scalar>(children.size());
            direction = mean_pos - center;
            direction.normalize();
        } else {
            // no children - fallback to the global Y-axis of this joint
            const auto& M = joint->global_initial_;
            // Y-axis is second column of rotation part
            direction = pmp::vec3 { M(0,1), M(1,1 ), M(2,1) };
            direction.normalize();
        }

        // build a line from center
        joint_center_lines.emplace_back(center, direction);
    }

    for (int i = 0; i < static_cast<int>(joint_center_lines.size()); ++i) {
        const auto& jcl = joint_center_lines[i];
        std::cout << "joint=" << rig.joints_[i]->name_
            << "\t center=" << std::get<0>(jcl)
            << "\t direction=" << std::get<1>(jcl)
            << std::endl;
    }

    return joint_center_lines;
}

// ---------------------------------------------------------------------------------------------------------------------

auto closest_point_on_line(const pmp::vec3& point, const Line& line) -> pmp::vec3
{
    // origin and direction for the line
    // const auto&[orig, dir] = line;
    const auto& o = pmp::vec3(std::get<0>(line));
    const auto d = pmp::vec3(std::get<1>(line));

    // normalize to simplfy
    normalize(d);

    // distance on line, ugly conversion to scalar
    // const auto part = ((transpose(orig - point) * dir) / (transpose(dir) * dir)[0])[0];
    const auto t=  (transpose(point - o) * d)[0];

    // closest point on line to point
    return o + t * d;
}

// ---------------------------------------------------------------------------------------------------------------------

auto update_spherical_wrap_parameters(VolumetricState* state) -> void
{
    // reset parameters
    auto& result = state->get_skel_wrap_joint_spheres();
    result.clear();

    // use an unordered set
    const auto& collapsed_vertices = std::unordered_set<unsigned int>(state->collapsed_vertices.begin(), state->collapsed_vertices.end());
    // get skinning parameters for skeleton wrap
    const auto& skinning_parameters = state->get_skinning_parameters(Layer::SKEL);

    // get joint center lines
    const auto joint_center_lines = get_joint_center_lines(state);

    // get current skeleton wrap
    const auto& skel_wrap = state->get_layers().to_mesh(Layer::SKEL);

    for (const auto vertex : skel_wrap.vertices()) {
        if (collapsed_vertices.contains(vertex.idx())) {
            continue;
        }

        // new entry for this vertex
        std::vector<SphereParams> sphere_params {};

        // use all skinning parameters >= EPS for distance and joint
        const auto& [v_depends, v_weights] = skinning_parameters[vertex.idx()];
        for (auto j = 0; j < 8; ++j) {
            // does bone v_deps[j] have an influence this vertex?
            if (v_depends[j] != 0 && v_weights[j] >= 1e-4) {
                const auto& j_idx = v_depends[j];
                if (j_idx < 0 || j_idx > static_cast<int> (joint_center_lines.size())) {
                    throw std::runtime_error("Joint index out of bounds for spherical params.");
                }
                const auto& line = joint_center_lines[j_idx];
                const auto& v_pos = skel_wrap.position(vertex);
                const auto& closest_point = closest_point_on_line(v_pos, line);
                sphere_params.emplace_back(j_idx, closest_point, pmp::distance(closest_point, v_pos));

                std::cout << "Vertex " << vertex.idx()
                    << "(" << result.size() - 1 << ")"
                    << "\tjoint=" << state->rig.joints_[j_idx]->name_
                    << "\tweight=" << v_weights[j]
                    << "(" << j_idx << ")"
                    << "\ton_line=" << closest_point
                    << "\tpos=" << v_pos
                    << "\tdist=" << pmp::distance(closest_point, v_pos) << std::endl;
            }
        }

        // non-collapsed vertex, add to params
        result.emplace_back(sphere_params);
    }

    std::cout << "UPDATED SPHERICAL WRAP PARAMETERS\n";
}


// ---------------------------------------------------------------------------------------------------------------------

auto RetargetSkeleton::setup_retargeting(VolumetricState* state) -> void
{
    if (state == nullptr) {
        throw std::runtime_error("setup_retargeting: state is null");
    }

    // use only skeleton wrap for MVC
    SurfaceMesh& skel_wrap = state->get_template_layer_unsafe(Layer::SKEL);

    if (skel_wrap.n_vertices() == 0) {
        std::cerr << "[ERROR] update_set_mvc(): Template wrap wrapping is empty.\n";
        return;
    }

    // calculate mean value coords
    mvc::compute_weights_from_current_joint_positions(state->rig, skel_wrap);
}

// ---------------------------------------------------------------------------------------------------------------------

auto RetargetSkeleton::retarget_skeleton(VolumetricState* state) -> void
{
    if (state == nullptr) {
        throw std::runtime_error("retarget_skeleton: state is null");
    }

    const auto& template_skel_wrap = state->get_template_layer(Layer::SKEL);
    if (not template_skel_wrap.has_vertex_property(mvc::MVC_PROPERTY_NAME)) {
        throw std::invalid_argument("template layer: skeleton mvc vertex property not found");
    }
    if (template_skel_wrap.n_vertices() == 0) {
        throw std::invalid_argument("template layer: template skeleton layer has no vertices.");
    }
    auto mvc_prop_source = template_skel_wrap.get_vertex_property<mvc::MeanValueCoords>(mvc::MVC_PROPERTY_NAME);

    // use current loaded skeleton wrap and retarget animation skeleton
    auto current_skel_wrap = state->get_layers().to_mesh(Layer::SKEL);

    if (current_skel_wrap.n_vertices() == 0) {
        std::cerr << "[ERROR] retarget_skeleton(): Current skeleton layer is empty.\n";
        return;
    }


    // copy mvc from template skeleton wrap to current layer
    auto mvc_prop_target = current_skel_wrap.vertex_property<mvc::MeanValueCoords>(mvc::MVC_PROPERTY_NAME);
    for (const auto vertex : current_skel_wrap.vertices()) {
        mvc_prop_target[vertex] = mvc_prop_source[vertex];
    }

    // update joint positions, based on skeleton wrap topology
    const auto new_joint_positions = mvc::recalculate_joint_positions(state->rig, current_skel_wrap);
    state->rig.update_joint_positions(new_joint_positions);
    state->rig.forward_kinematics();

    // restore bindpose (which is broken after retargeting)
    const auto bindpose_rotvec = state->get_bindpose();
    if (not bindpose_rotvec.empty()) {
        utils::StateModification::set_pose(state, bindpose_rotvec);
    }
    state->rig.set_current_pose_as_bindpose();

    // create new joint distances and joint centers for the skeleton wrap
    // update_spherical_wrap_parameters(state);
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(volumetric_base)

// =====================================================================================================================
