//
// Created by gerrit on 17.07.24.
//

#ifndef PRISM_BASED_COROTATIONAL_ELASTICITY_H
#define PRISM_BASED_COROTATIONAL_ELASTICITY_H

#include <Eigen/Sparse>
#ifdef USE_MKL
#include <Eigen/PardisoSupport>
#endif

#include <tuple>

#include "../base/basic_skinning.h"
#include "volumetric_base/layered_mesh.h"
#include "volumetric_character/volumetric_character.h"

typedef Eigen::SparseMatrix<double,EIGEN_DEFAULT_MATRIX_STORAGE_ORDER_OPTION,long long> longSparseMatrix;



class PrismBasedCorotationalElasticity {
public:
    explicit PrismBasedCorotationalElasticity(
      const volumetric_base::LayeredMesh &mesh);

    void setup_new_mesh(const volumetric_base::LayeredMesh &mesh);

    //Explicitly not possible because Eigen memory pointers get corrupted during copy
    PrismBasedCorotationalElasticity& operator=(const PrismBasedCorotationalElasticity& other) = delete;

    void animate(const volumetric_base::LayeredMesh& skeleton);
    void animate(volumetric_base::LayeredMesh &mesh, const volumetric_base::VolumetricState& state);
    [[nodiscard]] volumetric_base::LayeredMesh animate_copy(volumetric_base::LayeredMesh mesh,
                 const volumetric_base::VolumetricState& state);

    static std::tuple<double, double> get_material_parameters(const volumetric_base::Prism &cell);

    void precompute_stiffness_matrices(const volumetric_base::LayeredMesh &mesh);
    void compute_global_stiffness(const volumetric_base::LayeredMesh& mesh);
    void recompute_global_stiffness(const volumetric_base::LayeredMesh& mesh);
    void compute_righthandside(const volumetric_base::LayeredMesh &mesh);
    void compute_rotations_and_translations(const volumetric_base::LayeredMesh& mesh, const volumetric_base::Rig& rig);
    void set_sparse_matrix_entries(const volumetric_base::LayeredMesh &mesh);
    void computeIndexMapping(const volumetric_base::LayeredMesh &mesh);
    std::vector<Eigen::Matrix3d> rotations;
    std::vector<Eigen::Vector3d> translations;
    std::vector<Eigen::Matrix<double, 18, 18> > prism_stiffness_matrices;
    std::vector<double*> sparse_mat_entries;
    Eigen::Matrix<double, -1, 3> undeformed_positions;
    std::vector<Eigen::Triplet<double> > triplets;
    Eigen::VectorXd righthandside;
    std::vector<int> triplet_offsets;
    Eigen::SparseMatrix<double> global_k;
    std::vector<int> index_mapping;

    int number_of_dofs=0;
#ifdef USE_MKL
    Eigen::PardisoLDLT<longSparseMatrix> local_solver;
  #else
    Eigen::SimplicialLDLT<longSparseMatrix, Eigen::Upper> local_solver;
#endif
    bool preanalyzed=false;
};


#endif //PRISM_BASED_COROTATIONAL_ELASTICITY_H