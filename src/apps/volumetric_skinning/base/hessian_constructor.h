//
// Created by gerrit on 6/26/25.
//

#ifndef HESSIAN_CONSTRUCTOR_H
#define HESSIAN_CONSTRUCTOR_H
#include "Eigen/Sparse"
#include "volanim_utils/defs.h"
#include "volumetric_base/utility/matrix_util.hpp"

NAMESPACE_BEGIN(volumetric_skinning::newton)
    template <int dim>
class HessianConstructor {
    public:
        explicit HessianConstructor(std::vector<int> constrained_mapping );
        HessianConstructor() = default;
        void setup_sparse_matrix_entries(std::vector<std::array<int,dim>> elems);
        Eigen::SparseMatrix<double> assemble_hessian(std::function<Eigen::Matrix<double, dim*3,dim*3>(std::array<int, dim>, int)> hess_generator, std::vector<std::array<int, dim>>elems);
        Eigen::SparseMatrix<double> assemble_hessian_in_existing_memory(std::function<Eigen::Matrix<double, dim*3,dim*3>(std::array<int, dim>, int)> hess_generator, std::vector<std::array<int, dim>>elems);
        std::vector<Eigen::Matrix<double,3*dim,dim*3>> local_hessians;
        Eigen::SparseMatrix<double> hessian;
        std::vector<int> constrained_mapping;
        std::vector<double*> sparse_mat_entries;
        int num_dofs=0;
};

    template<int dim>
    void HessianConstructor<dim>::setup_sparse_matrix_entries(std::vector<std::array<int, dim>> elems) {
        sparse_mat_entries.clear();
        for (unsigned int i=0; i< elems.size(); i++) {
            auto elem = elems[i];
            for (int out=0; out< dim;out++) {
                if (constrained_mapping[elem[out]] == -1)
                    continue;
                for (int in=0; in< dim; in++) {
                    if (constrained_mapping[elem[in]] == -1)
                        continue;
                    matrix_util::triangularBlockAssignSetupForPointers<3,3>(constrained_mapping[elem[out]]*3,constrained_mapping[elem[in]]*3,sparse_mat_entries, hessian);
                }
            }
        }
    }

template<int dim>
Eigen::SparseMatrix<double> HessianConstructor<dim>::assemble_hessian(
    std::function<Eigen::Matrix<double, dim*3, dim*3>(std::array<int, dim>, int)> hess_generator,
    std::vector<std::array<int, dim>> elems) {
        if (hessian.rows() > 0) {
            return assemble_hessian_in_existing_memory(hess_generator, elems);
        }
        hessian = Eigen::SparseMatrix<double>(num_dofs*3, num_dofs*3);
        std::vector<Eigen::Triplet<double>> triplets;

        for (int i = 0; i < static_cast<int>(elems.size()); i++) {
            const auto& elem = elems[i];
            Eigen::Matrix<double, dim*3, dim*3> hess = hess_generator(elem,i);
            for (int out =0; out < dim; out++) {
                int out_idx = constrained_mapping[elem[out]];
                if (out_idx == -1)
                    continue;
                for (int in=0; in < dim; in++) {
                    int in_idx = constrained_mapping[elem[in]];
                    if (in_idx == -1)
                        continue;
                    Eigen::Matrix3d block = hess.block(out*3, in*3,3,3);
                    matrix_util::triangularBlockAssign(out_idx*3, in_idx*3,block, triplets);
                }
            }
        }
        hessian.setFromTriplets(triplets.begin(), triplets.end());
        hessian.makeCompressed();
        setup_sparse_matrix_entries(elems);

        return hessian;
}

template<int dim>
Eigen::SparseMatrix<double> HessianConstructor<dim>::assemble_hessian_in_existing_memory(
    std::function<Eigen::Matrix<double, dim * 3, dim * 3>(std::array<int, dim>, int)> hess_generator,
    std::vector<std::array<int, dim>> elems) {
        matrix_util::setSparseMatrixToZero(hessian);
        if (local_hessians.empty()) {
            local_hessians.resize(elems.size());
        }
        #pragma omp parallel for
        for (int i = 0; i < static_cast<int>(elems.size()); i++) {
            const auto& elem = elems[i];
            Eigen::Matrix<double, dim*3, dim*3> hess = hess_generator(elem,i);
            local_hessians[i] = hess;
        }
        int offset=0;
        for (unsigned int i=0;i < elems.size();i++) {
            const auto& elem = elems[i];
            Eigen::Matrix<double, dim*3, dim*3> hess = local_hessians[i];
            for (int out =0; out < dim; out++) {
                int out_idx = constrained_mapping[elem[out]];
                if (out_idx == -1)
                    continue;
                for (int in=0; in < dim; in++) {
                    int in_idx = constrained_mapping[elem[in]];
                    if (in_idx == -1)
                        continue;
                    Eigen::Matrix3d block = hess.block(out*3, in*3,3,3);
                    matrix_util::triangularBlockAssignIntoPointers<3,3>(out_idx*3, in_idx*3,block, offset, sparse_mat_entries);
                }
            }
        }
    return hessian;
}

template<int dim>
HessianConstructor<dim>::HessianConstructor(std::vector<int> constrained_mapping ) : constrained_mapping(std::move(constrained_mapping)) {
    num_dofs = 0;
    for (int idx : this->constrained_mapping) {
        if (idx != -1 )num_dofs++;
    }
}


NAMESPACE_END(volumetric_skinning::newton)


#endif //HESSIAN_CONSTRUCTOR_H
