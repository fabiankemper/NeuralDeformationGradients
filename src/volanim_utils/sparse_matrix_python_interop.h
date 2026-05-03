//
// Created by gerrit on 11/12/25.
//

#ifndef VOLUMETRIC_ANIMATION_SPARSE_MATRIX_PYTHON_INTEROP_H
#define VOLUMETRIC_ANIMATION_SPARSE_MATRIX_PYTHON_INTEROP_H
#include "unsupported/Eigen/src/SparseExtra/BlockOfDynamicSparseMatrix.h"
#include <Eigen/Sparse>
namespace util::io::sparse_py{
    void save_sparse_matrix(const Eigen::SparseMatrix<double>& matrix, const std::string& path);


}

#endif //VOLUMETRIC_ANIMATION_SPARSE_MATRIX_PYTHON_INTEROP_H