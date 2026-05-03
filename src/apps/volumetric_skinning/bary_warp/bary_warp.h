// =====================================================================================================================

#pragma once
#ifndef BARY_WARP_H
#define BARY_WARP_H

// =====================================================================================================================
#include <fstream>
#include <unordered_set>

#include <Eigen/Sparse>
#include <pmp/algorithms/barycentric_coordinates.h>
#include <pmp/algorithms/laplace.h>
#include <pmp/algorithms/numerics.h>

#include "pmp/stop_watch.h"
#include "pmp_ext/pmp_utility.h"
#include "pmp_ext/triangle_kd_tree.h"

#include "../base/collision_detection.h"
#include "../base/io.hpp"
#include "volumetric_base/layered_mesh.h"
#include "volumetric_base/skinnable_mesh.h"
#include "volumetric_base/state/filenames.h"
#include "volumetric_base/utility/precompute_solver.h"

// =====================================================================================================================

NAMESPACE_BEGIN(bary_warp)

// =====================================================================================================================

inline Eigen::MatrixXd vertex_mat(pmp::SurfaceMesh& mesh)
{
    Eigen::MatrixXd vertices(mesh.n_vertices(), 3);
    for (pmp::Vertex v: mesh.vertices()) {
        vertices.row(v.idx()) = static_cast<Eigen::Vector3d>(mesh.position(v));
    }
    return vertices;
}


inline Eigen::SparseMatrix<double> compute_steiner_mapping(pmp::SurfaceMesh& base_mesh, Eigen::MatrixXd points)
{
    //map skeleton wrap to skeleton wrap + steiner points
    pmp_ext::TriangleKDTree tree(&base_mesh);
    Eigen::SparseMatrix<double> A = Eigen::SparseMatrix<double>(points.rows(), base_mesh.n_vertices() * 3);
    std::vector<Eigen::Triplet<double> > triplets;
    for (int i = 0; i < points.rows(); ++i) {
        pmp::Point p = points.row(i);
        auto nearest = tree.nearest(p);
        pmp::Face f = nearest.face;

        std::vector<pmp::Vertex> vertices;
        pmp_ext::utils::getFaceVertices(base_mesh, f, vertices);


        pmp::Point barys = pmp::barycentric_coordinates(nearest.nearest, base_mesh.position(vertices[0]),
                                                        base_mesh.position(vertices[1]),
                                                        base_mesh.position(vertices[2]));
        triplets.emplace_back(i, vertices[0].idx() * 3, barys[0]);
        triplets.emplace_back(i, vertices[1].idx() * 3, barys[1]);
        triplets.emplace_back(i, vertices[2].idx() * 3, barys[2]);
    }
    A.setFromTriplets(triplets.begin(), triplets.end());
    return A;
}


inline std::vector<std::array<int, 4> > readTetsFromFile(const std::string& filename)
{
    std::vector<std::array<int, 4> > tets;
    std::ifstream infile(filename);
    if (!infile) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return tets;
    }

    std::string line;

    // Skip the first line
    std::getline(infile, line);

    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        int tetIndex, v0, v1, v2, v3;
        if (iss >> tetIndex >> v0 >> v1 >> v2 >> v3) {
            tets.push_back({v0, v1, v2, v3});
        }
    }

    return tets;
}

inline Eigen::Vector4d barycentricCoords(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& v1,
    const Eigen::Vector3d& v2,
    const Eigen::Vector3d& v3)
{
    Eigen::Matrix3d M;
    M.col(0) = v1 - v0;
    M.col(1) = v2 - v0;
    M.col(2) = v3 - v0;

    Eigen::Vector3d rhs = p - v0;
    Eigen::Vector3d w123 = M.fullPivLu().solve(rhs);

    double w0 = 1.0 - w123.sum();
    return {w0, w123[0], w123[1], w123[2]};
}


template<typename T, int j>
Eigen::Matrix<T, Eigen::Dynamic, j> eigen_from_vector(const std::vector<Eigen::Vector<T, j> >& vecs)
{
    Eigen::Matrix<T, Eigen::Dynamic, j> eig = Eigen::Matrix<T, Eigen::Dynamic, j>(static_cast<int>(vecs.size()), j);
    for (int k = 0; k < vecs.size(); k++) {
        eig.row(k) = vecs[k];
    }
    return eig;
}



inline Eigen::SparseMatrix<double> coefficients_to_sparse(const Eigen::VectorXi& muscle_indices,
                                                          const Eigen::MatrixXi& prism_indices,
                                                          const Eigen::MatrixXd& coeffs, int rows, int cols,
                                                          std::unordered_set<int>& found_Vertices)
    //Construct mapping of prism vertices -> hires muscle vertices from previously computed prism coeffs (in python)
{
    Eigen::SparseMatrix<double> res(rows, cols);
    std::vector<Eigen::Triplet<double> > triplets;
    for (int i = 0; i < coeffs.rows(); ++i) {
        int muscle_index = muscle_indices[i];
        if (found_Vertices.contains(muscle_index))
            continue;
        found_Vertices.emplace(muscle_index);

        Eigen::VectorXi indices = prism_indices.row(i);
        Eigen::VectorXd local_coeffs = coeffs.row(i);
        if (abs(local_coeffs.sum() - 1) > 1e-6) std::cout << "Coeffs dont sum to one" << std::endl;
        if (local_coeffs.array().minCoeff() < 0) std::cout << "Coeff less than one" << std::endl;
        for (int k = 0; k < 6; k++) {
            triplets.emplace_back(muscle_index, indices(k), local_coeffs(k));
        }
    }
    res.setFromTriplets(triplets.begin(), triplets.end());
    return res;
}


inline Eigen::SparseMatrix<double> compute_barycentric_coordinate_matrix(
    const volumetric_base::LayeredMesh& template_mesh, pmp::SurfaceMesh skel_mesh, Eigen::MatrixXd& positions,
    std::vector<std::tuple<pmp::Vertex, std::array<int, 4> > >& collisions)
{
    //represents things that are found within the skeleton wrap barycentrically using the tets from tetgen
    pmp::SurfaceMesh template_skel = template_mesh.to_mesh(volumetric_base::Layer::SKEL);
    Eigen::SparseMatrix<double> res(skel_mesh.n_vertices(), positions.rows());
    std::vector<Eigen::Triplet<double> > triplets;
    std::unordered_set<int> processed_indices;
    int maxindx = 0;
    for (auto [vertex,tet]: collisions) {
        auto [i1,i2,i3,i4] = tet;
        Eigen::Vector3d p1 = positions.row(i1);
        Eigen::Vector3d p2 = positions.row(i2);
        Eigen::Vector3d p3 = positions.row(i3);
        Eigen::Vector3d p4 = positions.row(i4);
        if (processed_indices.contains(vertex.idx())) continue;
        processed_indices.emplace(vertex.idx());
        Eigen::Vector3d pos = skel_mesh.position(vertex);
        auto barycoords = barycentricCoords(pos, p1, p2, p3, p4);
        for (int i = 0; i < 4; i++) {
            triplets.emplace_back(vertex.idx(), tet[i], barycoords(i));
            maxindx = std::max(maxindx, tet[i]);
        }
    }
    res.setFromTriplets(triplets.begin(), triplets.end());
    return res;
}


inline Eigen::SparseMatrix<double> compute_matrix_muscle_in_skeleton(
    std::vector<std::tuple<pmp::Vertex, std::array<int, 4> > >& collisions,
    const pmp::SurfaceMesh& hires_muscle_mesh, const Eigen::MatrixXd& positions,
    std::unordered_set<int>& found_vertices)
{
    Eigen::SparseMatrix<double> A(hires_muscle_mesh.n_vertices(), positions.rows());
    std::vector<Eigen::Triplet<double> > triplets;
    for (const auto& [v, tet]: collisions) {
        if (found_vertices.contains(v.idx())) continue;
        found_vertices.emplace(v.idx());
        auto [i1,i2,i3,i4] = tet;;
        Eigen::Vector3d p1 = positions.row(i1);
        Eigen::Vector3d p2 = positions.row(i2);
        Eigen::Vector3d p3 = positions.row(i3);
        Eigen::Vector3d p4 = positions.row(i4);
        Eigen::Vector3d p = hires_muscle_mesh.position(v);
        Eigen::Vector4d bcoords = barycentricCoords(p, p1, p2, p3, p4);
        for (int i = 0; i < 4; i++) {
            triplets.emplace_back(v.idx(), tet[i], bcoords(i));
        }
    }
    A.setFromTriplets(triplets.begin(), triplets.end());
    return A;
}


inline void test_bary_coeffs(pmp::SurfaceMesh& hires_muscle_mesh, const volumetric_base::LayeredMesh& template_mesh,volumetric_base::Gender
    gender )
{
    Eigen::MatrixXd coeffs = util::io::load_as_npy(volanim::filenames::bary_coeffs(gender));
    Eigen::VectorXi muscle_indices = util::io::load_as_npyI(volanim::filenames::bary_muscle_indices(gender));
    Eigen::MatrixXi prism_indices = util::io::load_as_npyI(volanim::filenames::bary_prism_indices(gender));

    Eigen::MatrixXd posis = template_mesh.toVertexMatrix();

    for (int i = 0; i < 100; i++) {
        pmp::Vertex v(muscle_indices[i]);
        Eigen::Vector3d p = hires_muscle_mesh.position(v);
        Eigen::Vector3d p2 = Eigen::Vector3d::Zero();
        for (int j = 0; j < 6; j++) {
            Eigen::Vector3d pj = posis.row(prism_indices(i, j));
            p2 += pj * coeffs(i, j);
        }
        std::cout << p - p2 << std::endl;
    }
    exit(0);
}


inline std::tuple<Eigen::SparseMatrix<double>, std::unordered_set<int> > muscle_warp_matrix(
    const volumetric_base::LayeredMesh& template_mesh, pmp::SurfaceMesh hires_muscle_mesh, volumetric_base::Gender
    gender)
{
    //Computes the barycentric mapping from template vertices to the hires muscle mesh
    // std::cout << "Computing Muscle Warp Matrix for Gender " << (gender == volumetric_base::Gender::MALE? "male" :"female")<< std::endl;
    Eigen::SparseMatrix<double> laplace;
    pmp::laplace_matrix(hires_muscle_mesh, laplace);
    Eigen::MatrixXd coeffs = util::io::load_as_npy(volanim::filenames::bary_coeffs(gender));
    Eigen::VectorXi muscle_indices = util::io::load_as_npyI(volanim::filenames::bary_muscle_indices(gender));
    Eigen::MatrixXi prism_indices = util::io::load_as_npyI(volanim::filenames::bary_prism_indices(gender));
    Eigen::MatrixXd steiner_nodes = util::io::readTetgenNodeFile(volanim::filenames::bary_steiner_nodes(gender));
    auto tets = util::io::readTetsFromFile(volanim::filenames::bary_steiner_tets(gender));

    nonlinear_elasticity::collisions::CollisionDetection detector;
    detector.init(template_mesh);
    auto collisions = detector.test_for_collisions(template_mesh, hires_muscle_mesh);

    pmp::SurfaceMesh template_skel = template_mesh.to_mesh(volumetric_base::Layer::SKEL);
    Eigen::SparseMatrix<double> prolongation_mat = compute_steiner_mapping(template_skel, steiner_nodes);
    detector.init(template_mesh);
    auto muscle_in_skeleton = detector.test_for_collisions(hires_muscle_mesh, tets, steiner_nodes);
    std::unordered_set<int> found_vertices;

    Eigen::SparseMatrix<double> skel_mat = compute_matrix_muscle_in_skeleton(
                                               muscle_in_skeleton, hires_muscle_mesh, steiner_nodes,
                                               found_vertices) * prolongation_mat;
    Eigen::SparseMatrix<double> prism_mat = coefficients_to_sparse(muscle_indices, prism_indices, coeffs,
                                                                   hires_muscle_mesh.n_vertices(),
                                                                   template_mesh.n_layer_vertices(), found_vertices);
    return {skel_mat + prism_mat, found_vertices};
}

inline std::tuple<Eigen::SparseMatrix<double>, std::unordered_set<int> > skeleton_warp_matrix(
    const volumetric_base::LayeredMesh& template_mesh, pmp::SurfaceMesh hires_skel_mesh, volumetric_base::Gender gender)
    //Computes the barycentric mapping from template vertices to the hires skeleton mesh
{
    Eigen::MatrixXd steiner_nodes = util::io::readTetgenNodeFile(volanim::filenames::bary_steiner_nodes(gender));
    auto tets = util::io::readTetsFromFile(volanim::filenames::bary_steiner_tets(gender));

    pmp::SurfaceMesh template_skel = template_mesh.to_mesh(volumetric_base::Layer::SKEL);
    //map existing points to steiner points
    Eigen::SparseMatrix<double> prolongation_mat = compute_steiner_mapping(template_skel, steiner_nodes);
    nonlinear_elasticity::collisions::CollisionDetection detector;
    detector.init(template_mesh);
    auto collisions = detector.test_for_collisions(hires_skel_mesh, tets, steiner_nodes);
    std::unordered_set<int> found_vertices;
    for (auto [vert, tet]: collisions) {
        found_vertices.emplace(vert.idx());
    }
    Eigen::SparseMatrix<double> skelmat = compute_barycentric_coordinate_matrix(
        template_mesh, hires_skel_mesh, steiner_nodes, collisions);
    Eigen::SparseMatrix<double> skeleton_barycenter_matrix = skelmat * prolongation_mat;
    return {skeleton_barycenter_matrix, found_vertices};
}
struct WarpWeights {
    std::array<int,32> indices;
    std::array<double,32> weights;
};

class BarycentricWarper
{
public:
    BarycentricWarper(const volumetric_base::LayeredMesh& template_mesh, const pmp::SurfaceMesh& hires_skeleton,
                      const pmp::SurfaceMesh& hires_muscle, volumetric_base::Gender gender)
    {
        pmp::laplace_matrix(hires_skeleton, skeleton_laplace);
        pmp::laplace_matrix(hires_muscle, muscle_laplace);
        auto [skel_bary, skel_fixed] = skeleton_warp_matrix(template_mesh, hires_skeleton, gender);
        auto [muscle_bary, muscle_fixed] = muscle_warp_matrix(template_mesh, hires_muscle, gender);
        skeleton_warp = skel_bary;
        muscle_warp = muscle_bary;
        fixed_vertices_skeleton = skel_fixed;
        fixed_vertices_muscle = muscle_fixed;
        hires_muscle_mesh = hires_muscle;
        hires_skel_mesh = hires_skeleton;
        setup_warp_weights(hires_skel_mesh,skeleton_warp, skeleton_weights);
        setup_warp_weights(hires_muscle_mesh,muscle_warp, muscle_weights);
    }

    pmp::SurfaceMesh get_warped_skeleton(const volumetric_base::LayeredMesh& skinned_wraps) const {
        return get_warped_skeleton(skinned_wraps.toVertexMatrix());
    }

    static void setup_warp_weights(const pmp::SurfaceMesh& input_mesh,
                                   const Eigen::SparseMatrix<double>& mapping,
                                   std::vector<WarpWeights>& warp_weights);


    pmp::SurfaceMesh get_warped_skeleton(const Eigen::MatrixXd& vertex_matrix) const {
            pmp::SurfaceMesh skel_copy = hires_skel_mesh;
        pmp::StopWatch watch;
        watch.start();
    #pragma omp parallel for
            for (int i = 0; i < static_cast<int>(skeleton_weights.size()); ++i) {
                WarpWeights warp_weights = skeleton_weights[i];
                Eigen::Vector3d new_pos = Eigen::Vector3d::Zero();
                for (int k=0;k < 32; ++k) {
                    if (warp_weights.weights[k] == 0.0) break;
                    new_pos += warp_weights.weights[k] * vertex_matrix.row(warp_weights.indices[k]);

                }
                skel_copy.position(pmp::Vertex(i)) = new_pos;
            }
        watch.stop();
        std::cout << "Time for Skeleton Warp " << watch.elapsed()<< std::endl;
            return skel_copy;
    }

    void warp_skeleton(volumetric_base::SkinnableMesh &skeleton, const Eigen::MatrixXd& vertex_matrix) const {
        auto global_pos = skeleton._mesh.vertex_property<pmp::Point>(pmp_character::properties::vertexGlobalPosition);
#pragma omp parallel for schedule(static)
        for (int i = 0; i < static_cast<int>(skeleton_weights.size()); ++i) {
            WarpWeights warp_weights = skeleton_weights[i];
            Eigen::Vector3d new_pos = Eigen::Vector3d::Zero();
            for (int k=0;k < 32; ++k) {
                if (warp_weights.weights[k] == 0.0) break;
                new_pos += warp_weights.weights[k] * vertex_matrix.row(warp_weights.indices[k]);

            }
            global_pos[pmp::Vertex(i)] = new_pos;
        }
    }
    void warp_muscle(volumetric_base::SkinnableMesh &muscle, const Eigen::MatrixXd& vertex_matrix) const {

        auto global_pos = muscle._mesh.vertex_property<pmp::Point>(pmp_character::properties::vertexGlobalPosition);
#pragma omp parallel for schedule(static)
        for (int i = 0; i < static_cast<int>(muscle_weights.size()); ++i) {
            WarpWeights warp_weights = muscle_weights[i];
            Eigen::Vector3d new_pos = Eigen::Vector3d::Zero();
            for (int k=0;k < 32; ++k) {
                new_pos += warp_weights.weights[k] * vertex_matrix.row(warp_weights.indices[k]);
            }
            global_pos[pmp::Vertex(i)] = new_pos;
        }
    }

    pmp::SurfaceMesh get_warped_muscle(const volumetric_base::LayeredMesh& skinned_wraps) const {
        return get_warped_muscle(skinned_wraps.toVertexMatrix());
    }

    pmp::SurfaceMesh get_warped_muscle(const Eigen::MatrixXd& vertex_matrix) const {
        pmp::SurfaceMesh musc_copy = hires_muscle_mesh;
#pragma omp parallel for
        for (int i = 0; i < static_cast<int>(muscle_weights.size()); ++i) {
            WarpWeights warp_weights = muscle_weights[i];
            Eigen::Vector3d new_pos = Eigen::Vector3d::Zero();
            for (int k=0;k < 32; ++k) {
                new_pos += warp_weights.weights[k] * vertex_matrix.row(warp_weights.indices[k]);
            }
            musc_copy.position(pmp::Vertex(i)) = new_pos;
        }
        return musc_copy;
    }

    void smoothen_out_muscle(const std::unordered_set<int> &fixed_vertices, const Eigen::SparseMatrix<double>& laplace,
                                  pmp::SurfaceMesh& mesh)
    //Some vertices are not found due to numerical error, we interpolate those as a minimal area surface
    {
        Eigen::MatrixXd righthandside = Eigen::MatrixXd::Zero(mesh.n_vertices(), 3);

        Eigen::MatrixXd C = vertex_mat(mesh);
        Eigen::MatrixXd res = solver_muscle.solve(laplace,righthandside,[&](unsigned int i) -> bool { return fixed_vertices.contains(i);}, C);
        for (pmp::Vertex v: mesh.vertices()) {
            mesh.position(v) = res.row(v.idx());
        }
    }

    void smoothen_out_skeleton(const std::unordered_set<int> &fixed_vertices, const Eigen::SparseMatrix<double>& laplace,
                              pmp::SurfaceMesh& mesh)
    //Some vertices are not found due to numerical error, we interpolate those as a minimal area surface
    {
        Eigen::MatrixXd righthandside = Eigen::MatrixXd::Zero(mesh.n_vertices(), 3);

        Eigen::MatrixXd C = vertex_mat(mesh);
        Eigen::MatrixXd res = solver_skeleton.solve(laplace,righthandside,[&](unsigned int i) -> bool { return fixed_vertices.contains(i);}, C);
        for (pmp::Vertex v: mesh.vertices()) {
            mesh.position(v) = res.row(v.idx());
        }
    }
    std::vector<WarpWeights> skeleton_weights;
    std::vector<WarpWeights> muscle_weights;
protected:
    PrecomputeSolver solver_muscle, solver_skeleton;
    pmp::SurfaceMesh hires_skel_mesh, hires_muscle_mesh;
    Eigen::SparseMatrix<double> muscle_laplace, skeleton_laplace, muscle_warp, skeleton_warp;
    std::unordered_set<int> fixed_vertices_skeleton, fixed_vertices_muscle;


};

// =====================================================================================================================

NAMESPACE_END(bary_warp::)

// =====================================================================================================================

#endif //BARY_WARP_H

// =====================================================================================================================
