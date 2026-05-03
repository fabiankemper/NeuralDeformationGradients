// =====================================================================================================================

#include "prism_based_corotational_elasticity.h"

#include "../base/basic_skinning.h"
#include "volumetric_base/state/skinning/skinning.h"
#include "volumetric_skinning/corotational_linear_elasticity/linear_elasticity.hpp"
#include <pmp/stop_watch.h>
#include <pmp_character/DualQuaternion.h>

// =====================================================================================================================
using namespace Eigen;
using namespace volumetric_character;
using namespace volumetric_base;

// =====================================================================================================================

void PrismBasedCorotationalElasticity::compute_global_stiffness(const LayeredMesh& mesh)
{
    if (global_k.rows() == 0)
    {
        global_k.resize(number_of_dofs* 3, number_of_dofs* 3);
    }
    std::vector<Prism> cells = mesh.getPrisms();
    MatrixXd deformed_prolonged_positions = mesh.toVertexMatrix();
    triplets.clear();
    for (unsigned int cell_index = 0; cell_index < cells.size(); cell_index++)
    {
        const Prism& cell = cells[cell_index];
        auto [i1, i2, i3, i4, i5, i6] = mesh.getIndices(cell);
        triplet_offsets.emplace_back(static_cast<long>(triplets.size()));

        std::array indices = {
            index_mapping[i1], index_mapping[i2], index_mapping[i3], index_mapping[i4], index_mapping[i5],index_mapping[i6]
        };
        Matrix<double, 18, 18> cell_K = prism_stiffness_matrices[cell_index];
        const Matrix3d& rota = rotations[cell_index];
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (indices[i] == -1)
                {
                    continue;
                }
                if(indices[j]==-1)
                {
                    continue;
                }
                Matrix3d ijblock = rota * cell_K.block<3, 3>(i * 3, j * 3) * rota.transpose();
                matrix_util::triangularBlockAssign(indices[i] * 3, indices[j] * 3, ijblock, triplets);
            }
        }
    }
    global_k.setFromTriplets(triplets.begin(), triplets.end());
    set_sparse_matrix_entries(mesh);
}

// =====================================================================================================================

void PrismBasedCorotationalElasticity::recompute_global_stiffness(const LayeredMesh& mesh)
{
    if (global_k.rows() == 0)
    {
        global_k.resize(number_of_dofs* 3, number_of_dofs*3);
    }
    for(int i=0; i< global_k.outerSize(); i++)
    {
        for(SparseMatrix<double>::InnerIterator it(global_k, i); it; ++it)
        {
            it.valueRef()=0;
        }
    }
    std::vector<Prism> cells = mesh.getPrisms();
    MatrixXd deformed_prolonged_positions =  mesh.toVertexMatrix();
    std::array indices = {0,0,0,0,0,0};
    for (unsigned int cell_index = 0; cell_index < cells.size(); cell_index++)
    {
        const Prism& cell = cells[cell_index];
        const auto& [i1, i2, i3, i4, i5, i6] = mesh.getIndices(cell);
        int offset = triplet_offsets[cell_index];
        indices[0] = index_mapping[i1];
        indices[1] = index_mapping[i2];
        indices[2] = index_mapping[i3];
        indices[3] = index_mapping[i4];
        indices[4] = index_mapping[i5];
        indices[5] = index_mapping[i6];

        const Matrix<double, 18, 18>& cell_K = prism_stiffness_matrices[cell_index];
        const Matrix3d& rota = rotations[cell_index];
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (indices[i] == -1)
                {
                    continue;
                }
                Matrix3d ijblock = rota * cell_K.block<3, 3>(i * 3, j * 3) * rota.transpose();
                if(indices[i]< indices[j])
                {
                    *sparse_mat_entries[offset+0] += ijblock(0,0);
                    *sparse_mat_entries[offset+1] += ijblock(0,1);
                    *sparse_mat_entries[offset+2] += ijblock(0,2);
                    *sparse_mat_entries[offset+3] += ijblock(1,0);
                    *sparse_mat_entries[offset+4] += ijblock(1,1);
                    *sparse_mat_entries[offset+5] += ijblock(1,2);
                    *sparse_mat_entries[offset+6] += ijblock(2,0);
                    *sparse_mat_entries[offset+7] += ijblock(2,1);
                    *sparse_mat_entries[offset+8] += ijblock(2,2);
                    offset +=9;
                }
                else if(indices[i] == indices[j])
                {
                    *sparse_mat_entries[offset+0] += ijblock(0,0);
                    *sparse_mat_entries[offset+1] += ijblock(0,1);
                    *sparse_mat_entries[offset+2] += ijblock(0,2);
                    *sparse_mat_entries[offset+3] += ijblock(1,1);
                    *sparse_mat_entries[offset+4] += ijblock(1,2);
                    *sparse_mat_entries[offset+5] += ijblock(2,2);
                    offset += 6;
                }
            }
        }
    }
}

// =====================================================================================================================

void PrismBasedCorotationalElasticity::compute_rotations_and_translations(
    const LayeredMesh& mesh, const Rig& rig
) {
    using namespace std;
    using namespace pmp;
    using namespace Eigen;
    using dualquat = Eigen::Vector<double, 8>;

    std::vector<Prism> cells = mesh.getPrisms();
    if(rotations.empty()) {
        rotations = vector<Matrix3d>(cells.size());
        translations = vector<Vector3d>(cells.size());
    }

    auto blend_dual_quats = [&](const vector<dualquat> &dualquats,
                                const vector<double> &weights)-> tuple<Matrix3d, Vector3d> {
        dualquat quatsum = dualquat::Zero();
        auto get_first_quat = [&](dualquat &quat)-> Vector4d { return {quat[0], quat[1], quat[2], quat[3]}; };
        for (unsigned int i = 0; i < dualquats.size(); i++) {
            dualquat temp = dualquats[i];
            double dotprod = get_first_quat(quatsum).dot(get_first_quat(temp));
            dualquat add_val = temp * weights[i];
            quatsum += dotprod >= 0 ? add_val : -add_val;
        }
        quatsum /= get_first_quat(quatsum).norm();
        auto transmatrix = static_cast<Matrix4d>(pmp_character::dualquat_to_mat4(quatsum));
        Matrix3d rota = transmatrix.block<3, 3>(0, 0);
        Vector3d translation = transmatrix.block<3, 1>(0, 3);
        return {rota, translation};
    };
    VertexProperty<ivec8> deps = mesh.get_vertex_property<ivec8>(VP_DEPENDS);
    VertexProperty<vec8> skinning_weights = mesh.get_vertex_property<vec8>(
        VP_SKINNING_WEIGHTS);
   rig.forward_kinematics();
    std::vector<mat4> _skinning_matrices;
    _skinning_matrices.resize(rig.joints_.size());
    for (unsigned int i = 0; i < rig.joints_.size(); ++i) {
        _skinning_matrices[i] = rig.joints_[i]->transformation_;
    }
#pragma omp parallel for
    for (int cell_idx = 0; cell_idx < static_cast<int>(cells.size()); cell_idx++) {
        const Prism &cell = cells[cell_idx];
        auto &[face, type] = cell;
        vector<dualquat> skinning_dualquats;
        vector<double> weights;
        auto [v1,v2,v3] = getFaceVertices(mesh, face);
        vector verts = {v1, v2, v3};
        for (Vertex v: verts) {
            for (int i = 0; i < 8; i++) {
                int depd = deps[v][i];
                double weight = skinning_weights[v][i];
                weights.emplace_back(weight);
                auto skinning_matrix = static_cast<Matrix4d>(_skinning_matrices[depd]);
                skinning_dualquats.emplace_back(
                    static_cast<dualquat>(pmp_character::mat4_to_dualquat(skinning_matrix)));
            }
        }
        auto [r,t] = blend_dual_quats(skinning_dualquats, weights);
        rotations[cell_idx] = r;
        translations[cell_idx] = t;
    }
}

// =====================================================================================================================

/*
 *  Store the memory positions of every entry in the stiffness matrix.
 *  Allows us to write directly into sparse matrix memory instead of
 *  building from Triplets at each step.
 */
void PrismBasedCorotationalElasticity::set_sparse_matrix_entries(const LayeredMesh& mesh)
{    sparse_mat_entries = std::vector<double*>(triplets.size());

    if (global_k.rows() == 0)
    {
        global_k.resize(static_cast<int>(mesh.n_vertices() * 2 * 3), static_cast<int>(mesh.n_vertices() * 2 * 3));
    }

    std::vector<Prism> cells = mesh.getPrisms();
    MatrixXd deformed_prolonged_positions = mesh.toVertexMatrix();
    for (unsigned int cell_index = 0; cell_index < cells.size(); cell_index++)
    {
        const Prism& cell = cells[cell_index];
        auto [i1, i2, i3, i4, i5, i6] = mesh.getIndices(cell);
        std::vector unmapped_indices = {i1, i2, i3, i4, i5, i6};

        std::vector indices = {
            index_mapping[i1], index_mapping[i2], index_mapping[i3], index_mapping[i4], index_mapping[i5],index_mapping[i6]
        };
        int offset = triplet_offsets[cell_index];

        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (indices[i] == -1)
                {
                    continue;
                }
                for(int k=0; k<3; k++)
                {
                    for(int l=0; l<3;l++)
                    {
                        if(indices[i]*3+k <= indices[j]*3+l)
                        {
                            sparse_mat_entries[offset]= &global_k.coeffRef(indices[i]*3+k, indices[j]*3+l);
                            offset++;
                        }
                    }
                }
            }
        }
    }
}

// =====================================================================================================================

void PrismBasedCorotationalElasticity::computeIndexMapping(const LayeredMesh& mesh)
{
    index_mapping = std::vector<int>(mesh.n_layer_vertices());
    int current_index = 0;
    for (auto [vertex, layer] : mesh.getLayerVertices())
    {
        index_mapping[LayeredMesh::Index(vertex, layer)] = layer==Layer::SKEL ? -1 : current_index++;

    }

    number_of_dofs = mesh.n_layer_vertices()-mesh.n_vertices();
}

// =====================================================================================================================

PrismBasedCorotationalElasticity::PrismBasedCorotationalElasticity(
    const LayeredMesh & mesh)
{
    undeformed_positions = mesh.toVertexMatrix();
    triplets = std::vector<Triplet<double>>(mesh.getPrisms().size() * 18 * 18);
    precompute_stiffness_matrices(mesh);
    computeIndexMapping(mesh);
}

// =====================================================================================================================

void PrismBasedCorotationalElasticity::setup_new_mesh(const LayeredMesh & mesh)
{
    precompute_stiffness_matrices(mesh);
    computeIndexMapping(mesh);
    undeformed_positions = mesh.toVertexMatrix();

    global_k.resize(0, 0);
    triplets.clear();
    triplet_offsets.clear();
    sparse_mat_entries.clear();

    preanalyzed = false;
}


// =====================================================================================================================

void PrismBasedCorotationalElasticity::animate(const LayeredMesh& mesh)
{
    if(righthandside.rows() ==0)
        righthandside.resize(number_of_dofs * 3, 1);

    if (global_k.rows() == 0)
        compute_global_stiffness(mesh);
    else
        recompute_global_stiffness(mesh);


    compute_righthandside(mesh);

    if (!preanalyzed)
    {
        local_solver.analyzePattern(global_k);
        preanalyzed = true;

    }
    local_solver.factorize(global_k);
    VectorXd results = local_solver.solve(righthandside);
    int idx = 0;

    for (auto [vertex, layer] : mesh.getLayerVertices())
    {
        if (layer != Layer::SKEL)
        {
            mesh.position(vertex, layer) = results.block<3, 1>(idx, 0);
            idx += 3;
        }
    }
}

// =====================================================================================================================

void PrismBasedCorotationalElasticity::animate(LayeredMesh& mesh,
                                               const volumetric_base::VolumetricState& state)
{
    compute_rotations_and_translations(mesh, state.rig);
    volumetric_skinning::basic_skinning::linear_blend_skin(mesh,&state);
    animate(mesh);
}

// =====================================================================================================================

LayeredMesh PrismBasedCorotationalElasticity::animate_copy(LayeredMesh mesh, const volumetric_base::VolumetricState& state)
{
    animate(mesh, state);
    return mesh;
}

// =====================================================================================================================

inline std::tuple<double, double> PrismBasedCorotationalElasticity::get_material_parameters(
    const Prism &cell) {
    return get<1>(cell) == Layer::SKEL ? muscle_params : skin_params;
}

// =====================================================================================================================

void PrismBasedCorotationalElasticity::compute_righthandside(const LayeredMesh& mesh)
{
    using namespace std;
    righthandside.setZero();
    std::vector<Prism> cells = mesh.getPrisms();
    MatrixXd deformed_positions =  mesh.toVertexMatrix();
    for (unsigned int cell_index = 0; cell_index < cells.size(); cell_index++)
    {
        const Prism& cell = cells[cell_index];
        const auto& [i1,i2,i3,i4,i5,i6] = mesh.getIndices(cell);
        const Matrix<double, 18, 18>& cell_K = prism_stiffness_matrices[cell_index];
        const Matrix3d& rota = rotations[cell_index];
        const Vector3d& translation = translations[cell_index];

        const std::array indices = {index_mapping[i1], index_mapping[i2], index_mapping[i3], index_mapping[i4], index_mapping[i5], index_mapping[i6]};
        const std::array unmapped_indices = {i1, i2, i3, i4, i5, i6};
        Vector3d rota_times_translation = (rota.transpose() * translation);
        for (int i = 0; i < 6; i++)
        {
            if (indices[i] == -1)
            {
                continue;
            }
            for (int j = 0; j < 6; j++)
            {
                Matrix3d RK = rota * cell_K.block<3, 3>(i * 3, j * 3);
                righthandside.block<3, 1>(indices[i] * 3, 0) += RK *
                (rota_times_translation + undeformed_positions.row(unmapped_indices[j]).
                    transpose());
                if (indices[j] == -1)
                {
                    righthandside.block<3, 1>(indices[i] * 3, 0) -= RK * rota.
                        transpose() * deformed_positions.row(unmapped_indices[j]).transpose();
                }
            }
        }
    }
}

// =====================================================================================================================

void PrismBasedCorotationalElasticity::precompute_stiffness_matrices(const LayeredMesh &mesh) {
    pmp::StopWatch stopwatch;

    stopwatch.start();
    prism_stiffness_matrices.clear();
    std::vector<Prism> cells =  mesh.getPrisms();
    prism_stiffness_matrices.resize(cells.size());
    using namespace std;
    for (unsigned int i=0; i< cells.size(); i++) {
        Prism &cell = cells[i];
        const auto& [nu, e] = get_material_parameters(cell);
        Matrix<double,3,6> positions = mesh.getPositionsAsMatrix(cell);
        this->prism_stiffness_matrices[i]=compute_prism_stiffness_matrix(positions, e,nu);

    }
    stopwatch.stop();
}
