//
// Created by gerrit on 4/2/25.
//

#ifndef LEAST_SQUARES_DEFGRADS_H
#define LEAST_SQUARES_DEFGRADS_H
#include <Eigen/Core>
#include <Eigen/Sparse>

#include "volumetric_base/layered_mesh.h"

namespace prism_diffgeo {
Eigen::Matrix<double, 3, 6>
prism_gradient(const Eigen::Matrix<double, 3, 6> &points);
Eigen::SparseMatrix<double>
gradientMatrix(const volumetric_base::LayeredMesh &mesh);
Eigen::SparseMatrix<double>
volumeMatrixCells(volumetric_base::LayeredMesh &mesh);

double prism_volume(Eigen::Matrix<double, 3, 6> P);
} // namespace prism_diffgeo

namespace volumetric_base::geometry {
// =====================================================================================================================

// ---------------------------------------------------------------------------------------------------------------------

double volume_tetrahedron_signed(const Eigen::Vector3d &i,
                                        const Eigen::Vector3d &j,
                                        const Eigen::Vector3d &k,
                                        const Eigen::Vector3d &l);
} // namespace volumetric_base::geometry

#endif // LEAST_SQUARES_DEFGRADS_H
