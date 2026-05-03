//
// Created by gerrit on 7/1/25.
//
#include <volumetric_base/layered_mesh.h>

#include "bary_warp.h"
#include "pmp_character/fbxfile/fbx_file.h"
const std::string male_muscle_fbx = "data/template/m_musculature.fbx";
const std::string male_skel_fbx = "data/template/m_skeleton.fbx";
const std::string female_muscle_fbx = "data/template/f_musculature.fbx";
const std::string female_skel_fbx = "data/template/f_skeleton.fbx";


inline void write_muscle_correspondence_matrices(pmp::SurfaceMesh& muscle_mesh,
                                                 const volumetric_base::LayeredMesh& template_mesh, const std::string &gender)
{
    //This is PREPROCESSING ONLY!! have to call this to provide data to the solver in python
    //solve_prism_barywarp_coordinates.py then computes and stores the indices
    std::vector<Eigen::Vector<int, 6> > prism_indices;
    std::vector<Eigen::Vector<int, 1> > vertex_indices;
    std::vector<Eigen::VectorXd> prism_positions;
    std::vector<Eigen::VectorXd> vertex_positions;
    std::unordered_set<int> found_verts;
    nonlinear_elasticity::collisions::CollisionDetection detector;
    detector.init(template_mesh);
    auto collisions = detector.test_for_collisions(template_mesh, muscle_mesh);
    for (const auto [vert ,tet, prism]: collisions) {
        auto [i1,i2,i3,i4,i5,i6] = template_mesh.getIndices(prism);
        Eigen::Vector<int, 6> indices;
        indices << i1, i2, i3, i4, i5, i6;
        prism_indices.emplace_back(indices);
        vertex_indices.emplace_back(vert.idx());
    }
    Eigen::MatrixXi prism_index_mat = bary_warp::eigen_from_vector<int, 6>(prism_indices);
    Eigen::VectorXi vertex_index_mat = bary_warp::eigen_from_vector<int, 1>(vertex_indices);
    Eigen::MatrixXd positions = template_mesh.toVertexMatrix();
    Eigen::MatrixXd vertices = bary_warp::vertex_mat(muscle_mesh);
    util::io::store_as_npy(positions, "data/bary_warp/" +gender+ "_template_positions.npy");
    util::io::store_as_npy(vertices, "data/bary_warp/" +gender+ "_muscle_positions.npy");
    util::io::store_as_npyI(vertex_index_mat, "data/bary_warp/" +gender+ "_muscle_indices.npy");
    util::io::store_as_npyI(prism_index_mat, "data/bary_warp/" +gender+ "_prism_indices.npy");
    std::cout << "Wrote Matrices for " << vertex_index_mat.rows() << " found verts" << std::endl;
}

int main(int argc, char** argv) {
    volumetric_base::LayeredMesh male_template = volumetric_base::LayeredMesh::template_mesh(volumetric_base::Gender::MALE);
    volumetric_base::LayeredMesh female_template = volumetric_base::LayeredMesh::template_mesh(volumetric_base::Gender::FEMALE);
    pmp_character::Character male_muscle, male_skeleton, female_muscle, female_skeleton;
    pmp_character::FBX_file file;
    file.read(&male_muscle, male_muscle_fbx );
    file.read(&male_skeleton, male_skel_fbx);
    file.read(&female_muscle, female_muscle_fbx );
    file.read(&female_skeleton, female_skel_fbx);
    pmp::SurfaceMesh male_hires_muscle_mesh = male_muscle.get_biggest_mesh().get()->mesh;
    pmp::SurfaceMesh male_hires_skel_mesh = male_skeleton.get_biggest_mesh().get()->mesh;
    pmp::SurfaceMesh female_hires_muscle_mesh = female_muscle.get_biggest_mesh().get()->mesh;
    pmp::SurfaceMesh female_hires_skel_mesh = female_skeleton.get_biggest_mesh().get()->mesh;
    std::cout << "Searching for " << male_hires_muscle_mesh.n_vertices() << " vertices " << std::endl;
    std::cout << "Processing Male Mesh" << std::endl;
    write_muscle_correspondence_matrices(male_hires_muscle_mesh, male_template, "male");
    std::cout << "Processing Female Mesh" << std::endl;
    write_muscle_correspondence_matrices(female_hires_muscle_mesh, female_template, "female");

}
