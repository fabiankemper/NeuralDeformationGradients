//
// Created by gerri on 25/08/2024.
//

#ifndef TETRAHEDRAL_H
#define TETRAHEDRAL_H
#include <vector>

#include "volumetric_base/layered_mesh.h"
#include <Eigen/Sparse>
NAMESPACE_BEGIN(volumetric_base)


using Tetrahedron = std::tuple<int,int,int,int>;

NAMESPACE_BEGIN(tetrahedral)

Eigen::SparseMatrix<double> getGradientMatrix(LayeredMesh &mesh);
Eigen::SparseMatrix<double> getLaplacianMatrix(LayeredMesh &mesh);
Eigen::SparseMatrix<double> getDivergenceMatrix(LayeredMesh &mesh);
std::tuple<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>>  getDiffGeoMatrices(LayeredMesh &mesh);
Eigen::DiagonalMatrix<double, Eigen::Dynamic> getVolumeMatrix(LayeredMesh &mesh);
std::vector<Tetrahedron> getTetrahedra(const LayeredMesh &mesh);
std::vector<std::tuple<Tetrahedron,Prism>> getTetrahedraAndCells(const LayeredMesh &mesh);
std::vector<std::tuple<Tetrahedron, Prism>> getTetrahedraAndCellsSafe(const LayeredMesh &mesh);

NAMESPACE_END(tetrahedral)


NAMESPACE_END(volumetric_base)

#endif //TETRAHEDRAL_H
