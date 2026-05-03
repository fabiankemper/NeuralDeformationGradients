#include "sparse_matrix_python_interop.h"

#include "npy.hpp"


void util::io::sparse_py::save_sparse_matrix(const Eigen::SparseMatrix<double> & matrix, const std::string& path) {
    std::vector<int> rows, cols;
    std::vector<double> vals;

    for (int k=0; k< matrix.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(matrix, k); it; ++it) {
            rows.emplace_back(it.row());
            cols.emplace_back(it.col());
            vals.emplace_back(it.value());
        }
    }
    const std::vector<long long> shape = {matrix.rows(), matrix.cols()};
    const unsigned long arr_shape[] = { static_cast<unsigned long> (rows.size()) };
    constexpr unsigned long shape_shape[] = {2};

    npy::SaveArrayAsNumpy(path + "/row.npy", false,1, arr_shape, rows.data());
    npy::SaveArrayAsNumpy(path + "/col.npy", false,1,arr_shape, cols);
    npy::SaveArrayAsNumpy(path + "/data.npy", false,1,arr_shape, vals);
    npy::SaveArrayAsNumpy(path + "/shape.npy", false,1, shape_shape, shape);
}
