// =====================================================================================================================

#include "mean_value_coordinates.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(mvc)

// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

auto compute_weights(pmp::SurfaceMesh& mesh, const std::vector<pmp::Point>& points, const std::string& pos_prop) -> void
{
    auto mvc_prop = mesh.vertex_property<MeanValueCoords>(MVC_PROPERTY_NAME);

    auto vps = mesh.get_vertex_property<pmp::Point>(pos_prop);

    for (pmp::Vertex v : mesh.vertices())
    {
        mvc_prop[v].resize(points.size());
    }

    for (size_t i = 0; i < points.size(); ++i)
    {
        double w_sum = 0.0;

        for (pmp::Vertex v : mesh.vertices())
        {
            pmp::dvec3 v_i = static_cast<pmp::dvec3>(vps[v] - points[i]);

            double w_i = 0.0;
            const double r = 1.0 / norm(v_i);

            pmp::dvec3 e_i = normalize(v_i);

            //loop local neighborhood
            auto vj_iter = mesh.vertices(v);
            if (mesh.is_boundary(*vj_iter)) {
                continue;
            }
            auto vj_iter_end = vj_iter;

            // Neighborhood data
            pmp::dvec3 v_j, e_j, v_k, e_k, n_jk, n_ij, n_ki;

            do
            {
                v_j = static_cast<pmp::dvec3>(vps[*vj_iter] - points[i]);
                e_j = normalize(v_j);
                if (mesh.is_boundary(*vj_iter))
                {
                    w_i = 0.0;
                    break;
                }

                ++vj_iter; // !! next vertex from here on !!

                if (mesh.is_boundary(*vj_iter))
                {
                    w_i = 0.0;
                    break;
                }

                v_k = static_cast<pmp::dvec3>(vps[*vj_iter] - points[i]);
                e_k = normalize(v_k);

                double beta_jk = std::abs(acos(dot(e_j, e_k)));
                double beta_ij = std::abs(acos(dot(e_i, e_j)));
                double beta_ki = std::abs(acos(dot(e_k, e_i)));

                n_jk = normalize(cross(e_j, e_k));
                n_ij = normalize(cross(e_i, e_j));
                n_ki = normalize(cross(e_k, e_i));

                double w = beta_jk + beta_ij * dot(n_ij, n_jk) + beta_ki * dot(n_ki, n_jk);

                if (!(w == w))
                {
                    std::cerr << "[ERROR] compute_mean_value_coordinates: Weight is nan!" << std::endl;
                }

                w *= 1.0 / (2.0 * dot(e_i, n_jk));
                w_i += w;
            }
            while (vj_iter != vj_iter_end);

            w_i *= r;
            //if (w_i < 0){
            //   std::cerr << "w_i is smaller 0:" << w_i << std::endl;
            //}
            w_sum += w_i;
            mvc_prop[v][i] = w_i;
        }

        // normalization of all weights
        for (const auto v : mesh.vertices())
        {
            mvc_prop[v][i] /= w_sum;
        }

        pmp::Point p_comp(0.0);
        for (pmp::Vertex v : mesh.vertices())
        {
            p_comp += mvc_prop[v][i] * vps[v];
        }
        float d = distance(p_comp, points[i]);
        if (d > 0.001)
        {
            std::cerr << "MVC points difference: Distance = " << d << " | real position " << p_comp <<
                " . calc. position " << points[i] << std::endl;
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto compute_weights_from_current_joint_positions(const Skeleton& skeleton, SurfaceMesh& mesh, const std::string& pos_prop) -> void
{
    mesh.vertex_property<MeanValueCoords>(MVC_PROPERTY_NAME);
    // if (mvc_prop)
    // {
    //     std::cerr <<"compute_mean_value_coords_from_current_joint_positions: [INFO] Can't compute MVC since you already have them!" << std::endl;
    //     return false;
    // }

    std::vector<pmp::Point> joint_positions(skeleton.joints_.size());
    for (size_t i = 0; i < joint_positions.size(); ++i)
    {
        joint_positions[i] = skeleton.joints_[i]->get_global_translation();
    }

    compute_weights(mesh, joint_positions, pos_prop);
}

// ---------------------------------------------------------------------------------------------------------------------

auto compute_new_joint_positions(pmp::SurfaceMesh& mesh, std::vector<pmp::Point>& points, const std::string& pos_prop) -> void
{
    // note: this function never returns false
    auto mvc_prop = mesh.vertex_property<MeanValueCoords>(MVC_PROPERTY_NAME);

    auto vpoint = mesh.vertex_property<pmp::Point>(pos_prop);

    // if (!mvc_prop || !vpoint)
    // {
    //     return false;
    // }

    points.clear();
    points.resize(mesh.n_vertices());

    for (size_t i = 0; i < points.size(); ++i)
    {
        auto p = pmp::Point(0.0f);
        for (pmp::Vertex v : mesh.vertices())
        {
            p += mvc_prop[v][i] * vpoint[v];
        }

        points[i] = p;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

auto recalculate_joint_positions(const Skeleton& skeleton, const SurfaceMesh& mesh) -> std::vector<pmp::Point>
{
    std::vector<pmp::Point> newPositions;

    auto mvc = mesh.get_vertex_property<MeanValueCoords>(MVC_PROPERTY_NAME);
    for (size_t i = 0; i < skeleton.joints_.size(); ++i)
    {
        // Calculate the global position
        pmp::Point pos(0);
        for (auto v : mesh.vertices())
        {
            pos += mesh.position(v) * mvc[v][i];
        }
        newPositions.emplace_back(pos);
    }
    return newPositions;
}

// ---------------------------------------------------------------------------------------------------------------------

auto fit_rig_to_mesh(Skeleton& skeleton, const SurfaceMesh& source_mesh, const SurfaceMesh& target_mesh) -> void
{
    if (source_mesh.n_vertices() != target_mesh.n_vertices()) {
        throw std::runtime_error("Source and target do not have same topology;");
    }

    // copy source for new vertex property
    pmp::SurfaceMesh source_mesh_temp = source_mesh;
    pmp::SurfaceMesh target_mesh_temp = target_mesh;

    // compute current weights
    compute_weights_from_current_joint_positions(skeleton, source_mesh_temp);

    // copy weights from source to target
    auto source_mvc_prop = source_mesh_temp.vertex_property<MeanValueCoords>(MVC_PROPERTY_NAME);
    auto target_mvc_prop = target_mesh_temp.vertex_property<MeanValueCoords>(MVC_PROPERTY_NAME);
    for (const auto vertex : source_mesh_temp.vertices()) {
        target_mvc_prop[vertex] = source_mvc_prop[vertex];
    }

    // get new positions based on calculated mvc's
    const auto new_joint_positions = recalculate_joint_positions(skeleton, target_mesh_temp);

    // apply new positions to skeleton
    skeleton.update_joint_positions(new_joint_positions);
}

// ---------------------------------------------------------------------------------------------------------------------

// =====================================================================================================================

NAMESPACE_END(mvc)
NAMESPACE_END(volumetric_base)

// =====================================================================================================================
