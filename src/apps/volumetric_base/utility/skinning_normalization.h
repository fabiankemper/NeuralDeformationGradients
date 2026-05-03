#ifndef SKINNING_NORMALIZATION_H
#define SKINNING_NORMALIZATION_H

#include "volanim_utils/defs.h"
#include <Eigen/Core>
#include <vector>
#include <array>
#include "pmp_character/DualQuaternion.h"
#include "volumetric_base/layered_mesh.h"
#include "volumetric_character/volumetric_character.h"
#include "pmp/stop_watch.h"

NAMESPACE_BEGIN(volumetric_base)
NAMESPACE_BEGIN(normalization)

typedef Eigen::Vector<double, 8> DualQuaternion;

// =====================================================================================================================
// LEGACY METHODS (Kept for backward compatibility)
// =====================================================================================================================
inline Eigen::Matrix3d blend_dual_quats(const std::vector<DualQuaternion> &dualquats,
                                        const std::vector<double> &weights) {
    DualQuaternion quatsum = DualQuaternion::Zero();
    auto get_first_quat = [&](DualQuaternion &quat)-> Eigen::Vector4d { return {quat[0], quat[1], quat[2], quat[3]}; };
    for (unsigned int i = 0; i < dualquats.size(); i++) {
        DualQuaternion temp = dualquats[i];
        double dotprod = get_first_quat(quatsum).dot(get_first_quat(temp));
        DualQuaternion add_val = temp * weights[i];
        quatsum += dotprod > 0 ? add_val : -add_val;
    }
    quatsum /= get_first_quat(quatsum).norm();
    auto transmatrix = static_cast<Eigen::Matrix4d>(pmp_character::dualquat_to_mat4(quatsum));
    Eigen::Matrix3d rota = transmatrix.block<3, 3>(0, 0);
    return rota;
}

inline Eigen::MatrixXd getRotationMatrix(const volumetric_character::VolumetricState& state, const LayeredMesh &mesh) {
    using namespace std;
    using namespace pmp;
    using namespace Eigen;
    using dualquat = Eigen::Vector<double, 8>;
    auto rig = state.rig;
    std::vector<Prism> cells = mesh.getPrisms();
    MatrixXd rotas(cells.size()*3,3);

    VertexProperty<ivec8> deps = mesh.get_vertex_property<ivec8>(VP_DEPENDS);
    VertexProperty<vec8> skinning_weights = mesh.get_vertex_property<vec8>(VP_SKINNING_WEIGHTS);
    rig.forward_kinematics();

    std::vector<dualquat> _skinning_dualquats;
    _skinning_dualquats.resize(rig.joints_.size());
    for (unsigned int i = 0; i < rig.joints_.size(); ++i) {
        _skinning_dualquats[i] = pmp_character::mat4_to_dualquat(rig.joints_[i]->transformation_);
    }

    OMP_PARALLEL_FOR()
    for (int cell_idx = 0; cell_idx < static_cast<int>(cells.size()); cell_idx++) {
        const Prism &cell = cells[cell_idx];
        auto &[face, type] = cell;
        vector<dualquat> skinning_dualquats(3*8);
        vector<double> weights(3*8);
        auto [v1,v2,v3] = getFaceVertices(mesh, face);
        array verts = {v1, v2, v3};
        for (int v_idx = 0; v_idx <3; ++v_idx) {
            Vertex v = verts[v_idx];
            for (int i = 0; i < 8; i++) {
                weights[v_idx*8+i] = skinning_weights[v][i];
                skinning_dualquats[v_idx*8+i] = _skinning_dualquats[deps[v][i]];
            }
        }
        rotas.block<3,3>(cell_idx*3,0) = blend_dual_quats(skinning_dualquats, weights);
    }
    return rotas;
}

// =====================================================================================================================
// OPTIMIZED HELPER: Computes the blended rotation for a single cell with ZERO allocations
// =====================================================================================================================
inline Eigen::Matrix3d compute_cell_rotation(
    const pmp::Vertex verts[3],
    const pmp::VertexProperty<ivec8>& deps,
    const pmp::VertexProperty<vec8>& weights_prop,
    const std::vector<Eigen::Matrix<double,8,1>>& jointDQ)
{
    using dualquat = Eigen::Matrix<double,8,1>;
    dualquat sum = dualquat::Zero();

    auto qreal = [](const dualquat& x){
        return Eigen::Map<const Eigen::Vector4d>(x.data());
    };

    // Use the first joint of the first vertex as our reference polarity
    Eigen::Vector4d qref = qreal(jointDQ[deps[verts[0]][0]]);

    for (int v = 0; v < 3; ++v) {
        const auto d = deps[verts[v]];
        const auto sw = weights_prop[verts[v]];

        for (int i = 0; i < 8; ++i) {
            const double weight = sw[i];

            // MASSIVE SPEEDUP: Skip math if the vertex isn't influenced by this joint slot
            if (weight > 1e-6) {
                const dualquat& dq_i = jointDQ[d[i]];
                const double dotprod = qref.dot(qreal(dq_i));
                const double s = (dotprod >= 0.0) ? 1.0 : -1.0;
                sum.noalias() += (s * weight) * dq_i;
            }
        }
    }

    // Normalize full dual quaternion
    const double nrm = qreal(sum).norm();
    if (nrm > 1e-12) sum /= nrm;

    // Recover rotation
    const Eigen::Matrix4d T = pmp_character::dualquat_to_mat4(sum);
    return T.block<3,3>(0,0);
}

// =====================================================================================================================
// OPTIMIZED: getRotationMatrixFast
// =====================================================================================================================
inline Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor>
getRotationMatrixFast(const volumetric_character::VolumetricState& state,
                      const LayeredMesh& mesh)
{
    using dualquat = Eigen::Matrix<double,8,1>;

    const auto& cells = mesh.getPrisms();
    Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> rotas(cells.size()*3,3);

    auto& rig = state.rig;
    rig.forward_kinematics();

    std::vector<dualquat> jointDQ(rig.joints_.size());
    for (size_t i=0; i<rig.joints_.size(); ++i)
        jointDQ[i] = pmp_character::mat4_to_dualquat(rig.joints_[i]->transformation_);

    auto deps   = mesh.get_vertex_property<ivec8>(VP_DEPENDS);
    auto weights_prop = mesh.get_vertex_property<vec8>(VP_SKINNING_WEIGHTS);

    #pragma omp parallel for schedule(static, 64)
    for (int cell_idx=0; cell_idx<(int)cells.size(); ++cell_idx) {
        const auto& [face,type] = cells[cell_idx];
        const auto [v1,v2,v3] = getFaceVertices(mesh, face);
        const pmp::Vertex verts[3] = {v1,v2,v3};

        rotas.block<3,3>(cell_idx*3,0) = compute_cell_rotation(verts, deps, weights_prop, jointDQ);
    }

    return rotas;
}

// =====================================================================================================================
// normalize_defgrads
// =====================================================================================================================
inline void normalize_defgrads(const VolumetricState& state, const LayeredMesh &mesh, Eigen::MatrixXd &grads)
{
    // Note: If this is used in a hot loop, you should switch it to use getRotationMatrixFast instead!
    Eigen::MatrixXd offsets = getRotationMatrix(state, mesh);

    for (int i=0; i< grads.rows() /3; i++ ) {
        const Eigen::Matrix3d& offset = offsets.block<3,3>(i*3,0);
        grads.block<3,3>(i*3, 0) = grads.block<3,3>(i*3,0)*offset;
    }
}

// =====================================================================================================================
// OPTIMIZED: unnormalize_defgrads
// =====================================================================================================================
inline void unnormalize_defgrads(const VolumetricState& state,
                                 const LayeredMesh& mesh,
                                 Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>& grads)
{
    pmp::StopWatch watch; watch.start();
    using dualquat = Eigen::Matrix<double,8,1>;

    const auto& cells = mesh.getPrisms();
    auto& rig = state.rig;
    rig.forward_kinematics();

    std::vector<dualquat> jointDQ(rig.joints_.size());
    for (size_t i=0; i<rig.joints_.size(); ++i)
        jointDQ[i] = pmp_character::mat4_to_dualquat(rig.joints_[i]->transformation_);

    auto deps   = mesh.get_vertex_property<ivec8>(VP_DEPENDS);
    auto weights_prop = mesh.get_vertex_property<vec8>(VP_SKINNING_WEIGHTS);

    #pragma omp parallel for schedule(static, 64)
    for (int cell_idx=0; cell_idx<(int)cells.size(); ++cell_idx) {
        const auto& [face,type] = cells[cell_idx];
        const auto [v1,v2,v3] = getFaceVertices(mesh, face);
        const pmp::Vertex verts[3] = {v1,v2,v3};

        const Eigen::Matrix3d R = compute_cell_rotation(verts, deps, weights_prop, jointDQ);

        // Use an explicit current block variable to prevent Eigen from evaluating a temporary
        Eigen::Matrix3d current = grads.block<3,3>(cell_idx*3,0);
        grads.block<3,3>(cell_idx*3,0).noalias() = current * R.transpose();
    }
    watch.stop();
    // std::cout << "Unnormalize rewrite: " << watch.elapsed() << "ms\n";
}

NAMESPACE_END(normalization)
NAMESPACE_END(volumetric_base)

#endif //SKINNING_NORMALIZATION_H