// =====================================================================================================================
//
// Created by gerri on 25/08/2024.
//
// =====================================================================================================================

#include "tetrahedral.h"

#include "prism_diffgeo.h"

#include <vector>

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_base)

// =====================================================================================================================

double getVolume(const Tetrahedron &tet, Eigen::MatrixXd &posis) {
    auto [i1,i2,i3,i4] = tet;
    Eigen::Vector3d v1 = posis.row(i1);
    Eigen::Vector3d v2 = posis.row(i2);
    Eigen::Vector3d v3 = posis.row(i3);
    Eigen::Vector3d v4 = posis.row(i4);
    return geometry::volume_tetrahedron_signed(v1,v2,v3,v4);
}


Eigen::Matrix<double,3,4> getTetGrad(const Tetrahedron& tet, const Eigen::MatrixXd &posis) {
    auto [i1,i2,i3,i4] = tet;

    Eigen::Matrix<double,3,3> edges;
    edges.row(0) = posis.row(i1) - posis.row(i4);
    edges.row(1) = posis.row(i2) - posis.row(i4);
    edges.row(2) = posis.row(i3) - posis.row(i4);
    Eigen::Matrix<double,3,4> coeffs;
    coeffs<<         1, 0, 0, -1,
                     0, 1 ,0, -1,
                     0, 0, 1, -1;
    return edges.inverse()*coeffs;
}

// =====================================================================================================================

NAMESPACE_BEGIN(tetrahedral)

// =====================================================================================================================

Eigen::SparseMatrix<double> getGradientMatrix(LayeredMesh &mesh) {
    using namespace Eigen;
    std::vector<Tetrahedron> tets = tetrahedral::getTetrahedra(mesh);
    Eigen::SparseMatrix<double> gradientMatrix(3*static_cast<int>(tets.size()), mesh.n_layer_vertices());
    Eigen::MatrixXd posis = mesh.toVertexMatrix();
    std::vector<Triplet<double>> triplets;
    for(unsigned int i=0; i<tets.size(); i++) {
        Tetrahedron & tet =tets[i];
        auto [i1,i2,i3,i4] = tet;
        std::vector vindices = {i1,i2,i3,i4};
        Matrix<double,3,4> gradMat = getTetGrad(tet, posis);
        for(int vertex=0; vertex< 4; vertex++) {
            int index = vindices[vertex];
            for(int k=0; k<3; k++) {
                triplets.emplace_back(i*3+k,index,gradMat(k,vertex));
            }
        }
    }
    gradientMatrix.setFromTriplets(triplets.begin(), triplets.end());
    return gradientMatrix;
}

Eigen::SparseMatrix<double> getLaplacianMatrix(LayeredMesh &mesh) {
    Eigen::SparseMatrix<double> grad = getGradientMatrix(mesh);
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> volumes = getVolumeMatrix(mesh);
    return grad.transpose()*volumes*grad;

}

Eigen::SparseMatrix<double> getDivergenceMatrix(LayeredMesh &mesh) {
    Eigen::SparseMatrix<double> gradient = getGradientMatrix(mesh);
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> volumes = getVolumeMatrix(mesh);
    return gradient.transpose()*volumes;
}

std::tuple<Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>, Eigen::SparseMatrix<double>>
getDiffGeoMatrices(LayeredMesh &mesh) {
    using namespace Eigen;
    SparseMatrix<double> gradient = getGradientMatrix(mesh);
    DiagonalMatrix<double, Eigen::Dynamic> volumes = getVolumeMatrix(mesh);
    SparseMatrix<double> divergence = gradient.transpose()*volumes;
    SparseMatrix<double> laplace = divergence*gradient;
    return {gradient, divergence, laplace};
}

Eigen::DiagonalMatrix<double, Eigen::Dynamic> getVolumeMatrix(LayeredMesh &mesh) {
    std::vector<Tetrahedron> tets = getTetrahedra(mesh);
    Eigen::VectorXd volumes(tets.size()*3);
    Eigen::MatrixXd posis = mesh.toVertexMatrix();
    for(unsigned int i=0; i<tets.size(); i++) {
        Tetrahedron tet = tets[i];
        double volume = getVolume(tet,posis);
        volumes.block<3,1>(i*3,0) = Eigen::Vector3d::Constant(volume);


    }
    return Eigen::DiagonalMatrix<double, Eigen::Dynamic>(volumes);
}

std::vector<Tetrahedron> getTetrahedra(const LayeredMesh &mesh) {
    using namespace Eigen;
    std::vector<Tetrahedron> tets;
    MatrixXd posis = mesh.toVertexMatrix();
    auto fix_posis = [&](int i1, int i2,int i3,int i4) -> Tetrahedron {

            Vector3d v1 = posis.row(i1);
            Vector3d v2 = posis.row(i2);
            Vector3d v3 = posis.row(i3);
            Vector3d v4 = posis.row(i4);
            double vol = geometry::volume_tetrahedron_signed(v1,v2,v3,v4) ;
            if(vol >0) {
                return Tetrahedron{i1,i2,i3,i4};
            }
        return  {i1,i2,i4,i3};
    };
    for(Prism cell: mesh.getPrisms()) {
        auto [i1,i2,i3,i4,i5,i6] = mesh.getIndices(cell);
        tets.emplace_back(fix_posis(i1,i2,i3,i4));
        tets.emplace_back(fix_posis(i2,i3,i4, i5));
        tets.emplace_back(fix_posis(i3,i4,i5,i6));
    }
    return tets;
}


std::vector<std::tuple<Tetrahedron, Prism>> getTetrahedraAndCellsSafe(const LayeredMesh &mesh) {
    using namespace Eigen;
    std::vector<std::tuple<Tetrahedron, Prism>> tets;
    MatrixXd posis = mesh.toVertexMatrix();

    auto fix_posis = [&](int i1, int i2, int i3, int i4) -> Tetrahedron {
        Vector3d v1 = posis.row(i1);
        Vector3d v2 = posis.row(i2);
        Vector3d v3 = posis.row(i3);
        Vector3d v4 = posis.row(i4);
        double vol = geometry::volume_tetrahedron_signed(v1, v2, v3, v4);
        if (vol > 0) {
            return Tetrahedron{i1, i2, i3, i4};
        }
        return Tetrahedron{i1, i2, i4, i3}; // flip orientation
    };

    for (Prism cell : mesh.getPrisms()) {
        auto [i1, i2, i3, i4, i5, i6] = mesh.getIndices(cell);
        tets.emplace_back(fix_posis(i1,i2,i3,i4), cell);
        tets.emplace_back(fix_posis(i2,i3,i5,i6), cell);
        tets.emplace_back(fix_posis(i2,i4,i5,i6), cell);
        tets.emplace_back(fix_posis(i1,i4,i5,i6), cell);
        tets.emplace_back(fix_posis(i1,i2,i5,i6), cell);
        tets.emplace_back(fix_posis(i1,i2,i3,i6), cell);



    }

    return tets;
}


std::vector<std::tuple<Tetrahedron, Prism>> getTetrahedraAndCells(const LayeredMesh& mesh)
{
    using namespace Eigen;
    std::vector<std::tuple<Tetrahedron, Prism>> tets;
    MatrixXd posis = mesh.toVertexMatrix();
    auto fix_posis = [&](int i1, int i2,int i3,int i4) -> Tetrahedron {

        Vector3d v1 = posis.row(i1);
        Vector3d v2 = posis.row(i2);
        Vector3d v3 = posis.row(i3);
        Vector3d v4 = posis.row(i4);
        double vol = geometry::volume_tetrahedron_signed(v1,v2,v3,v4) ;
        if(vol >0) {
            return Tetrahedron{i1,i2,i3,i4};
        }
        return  {i1,i2,i4,i3};
    };
    for(Prism cell: mesh.getPrisms()) {
        auto [i1,i2,i3,i4,i5,i6] = mesh.getIndices(cell);
        tets.emplace_back(fix_posis(i1,i2,i3,i4), cell);
        tets.emplace_back(fix_posis(i2,i3,i4, i5), cell);
        tets.emplace_back(fix_posis(i3,i4,i5,i6), cell);
    }
    return tets;

}

// =====================================================================================================================

NAMESPACE_END(tetrahedral)

// =====================================================================================================================

NAMESPACE_END(volumetric_base)

// =====================================================================================================================

//
// int main(int argc, char** argv) {
//     using namespace Eigen;
//     LayeredMesh meanMesh =DeformationTransfer::getMeanMesh();
//     auto tets = Tetrahedral::getTetrahedra(meanMesh);
//     MatrixXd posis = meanMesh.toVertexMatrixFast();
//     auto [grad, div, laplace] = Tetrahedral::getDiffGeoMatrices(meanMesh);
//     cout << grad.rows() << " " << grad.cols() << endl;
//     cout<< laplace*posis << endl;
// }