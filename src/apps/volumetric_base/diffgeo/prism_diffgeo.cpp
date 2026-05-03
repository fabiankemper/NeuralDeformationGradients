#include "prism_diffgeo.h"

#include <Eigen/Dense>

#include "volanim_utils/defs.h"
#include "volumetric_base/utility/matrix_util.hpp"

namespace prism_diffgeo {


    /*******************************************************************************************************************
     *
     *              Implements a prism gradient based on an integrated least square fit of
     *                                   points belonging to two prisms.
     *
     ******************************************************************************************************************/

    auto prism_volume(Eigen::Matrix<double, 3, 6> P) -> double
    {
        // P = prism
        // the first 3 points are for one tri, the second three points are the opposite tri.
        // this is compatible with the prism call in LayeredMesh
        auto detJ = [&](const double a, const double b, const double xi) -> double {
            const double g = 1.0 - a - b; // gamma by barycentric coordinates
            const Eigen::Vector3d dxa = xi * (P.col(0) - P.col(2)) + (1.0 - xi) * (P.col(3) - P.col(5));
            const Eigen::Vector3d dxb = xi * (P.col(1) - P.col(2)) + (1.0 - xi) * (P.col(4) - P.col(5));
            const Eigen::Vector3d dxi =  a * (P.col(0) - P.col(3)) +          b * (P.col(1) - P.col(4)) + g * (P.col(2) - P.col(5));
            return dxa.cross(dxb).dot(dxi);
        };

        // parameters derived by gauss-legendre integration
        // https://people.sc.fsu.edu/~jburkardt/datasets/quadrature_rules_tri/quadrature_rules_tri.html
        constexpr double a = 1.0 / 3.0;
        constexpr double b = 1.0 / 3.0;
        constexpr double sqrt3 = 1.7320508075688772;
        constexpr double xi1 = 0.5 * (1.0 - 1.0 / sqrt3);
        constexpr double xi2 = 0.5 * (1.0 + 1.0 / sqrt3);

        // integration sum
        const double v = 0.25 * ((detJ(a, b, xi1)) + (detJ(a, b, xi2)));
        return v;
    }

Eigen::SparseMatrix<double> volumeMatrixCells(volumetric_base::LayeredMesh & mesh)
    {
        std::vector<volumetric_base::Prism> cells = mesh.getPrisms();
        int number_of_prisms = static_cast<int>(cells.size());
        Eigen::SparseMatrix<double> volumeMat(3 * number_of_prisms, 3 * number_of_prisms);
        std::vector<Eigen::Triplet<double>> triplets;
        Eigen::Matrix<double, -1, 3> positions = mesh.toVertexMatrix();
        int tet_index = 0;

        for (volumetric_base::Prism cell : cells)
        {
            Eigen::Matrix<double,3,6> posis = mesh.getPositionsAsMatrix(cell);
            double volume = std::abs(prism_diffgeo::prism_volume(posis));

            Eigen::Matrix3d volMat = volume * Eigen::Matrix3d::Identity();
            matrix_util::blockAssign<3, 3>(tet_index, tet_index, volMat, triplets);
            tet_index += 3;
        }
        volumeMat.setFromTriplets(triplets.begin(), triplets.end());
        volumeMat.makeCompressed();
        return volumeMat;
    }

    Eigen::Matrix<double,3,6> prism_gradient(const Eigen::Matrix<double,3,6>& P)
    {
        // mean-center columns
        const Eigen::Vector3d mean = P.rowwise().mean();
        Eigen::Matrix<double,3,6> Pbar = P.colwise() - mean;

        // build W
        Eigen::Matrix<double,6,6> W = Eigen::Matrix<double,6,6>::Ones();
        for (int i = 0; i < 6; ++i) W(i,i) = 4.0;
        auto set_edge = [&](int i, int j){ W(i,j) = W(j,i) = 2.0; };
        set_edge(0,1); set_edge(1,2); set_edge(2,0); // base
        set_edge(3,4); set_edge(4,5); set_edge(5,3); // top
        set_edge(0,3); set_edge(1,4); set_edge(2,5); // vertical

        // S = Pbar W Pbar^T
        Eigen::Matrix3d S = Pbar * W * Pbar.transpose();

        // robust inverse
        Eigen::Matrix3d Sinv;
        Eigen::LDLT<Eigen::Matrix3d> ldlt(S);
        Sinv = ldlt.solve(Eigen::Matrix3d::Identity());

        const Eigen::Matrix<double,3,6> G = Sinv * Pbar * W;

        // center input
        const Eigen::Matrix<double,6,6> C = Eigen::Matrix<double,6,6>::Identity()
                                    - (1.0/6.0) * Eigen::Matrix<double,6,6>::Ones();

        return G * C;
    }


    // Collects prism gradients and gathers them into a large gradient matrix

    //Careful: Matrix expects vectors as rows and returns transposed gradients F^T
    Eigen::SparseMatrix<double> gradientMatrix(const volumetric_base::LayeredMesh& mesh) {
        Eigen::SparseMatrix<double> gradient(3*mesh.n_prisms(), mesh.n_layer_vertices());
        std::vector<Eigen::Triplet<double>> triplets(18*mesh.n_prisms());
        std::vector<volumetric_base::Prism> prisms = mesh.getPrisms();
        OMP_PARALLEL_FOR()
        for (int i=0; i< mesh.n_prisms(); i++) {
            int offset = 18*i;
            const volumetric_base::Prism &prism = prisms[i];
            const Eigen::Matrix<double,3,6> positions = mesh.getPositionsAsMatrix(prism);
            const auto & [v1,v2,v3,v4,v5,v6] = mesh.getIndices(prism);
            const std::vector indices = {v1,v2,v3,v4,v5,v6};
            const Eigen::Matrix<double,3,6> local_grad = prism_gradient(positions);
            for (int col = 0; col<6; col++) {
                triplets[offset++] = {i*3,indices[col],local_grad(0,col)};
                triplets[offset++] = {i*3+1,indices[col],local_grad(1,col)};
                triplets[offset++] = {i*3+2,indices[col],local_grad(2,col)};
            }
        }
        gradient.setFromTriplets(triplets.begin(), triplets.end());
        return gradient;
    }
} // namespace prism_diffgeo
    double volumetric_base::geometry::volume_tetrahedron_signed(
        const Eigen::Vector3d &i, const Eigen::Vector3d &j,
        const Eigen::Vector3d &k, const Eigen::Vector3d &l) {
    Eigen::Vector3d a, b, c, d;
    a << i[0], i[1], i[2];
    b << j[0], j[1], j[2];
    c << k[0], k[1], k[2];
    d << l[0], l[1], l[2];

    Eigen::Matrix3d A;
    A.col(0) = b - a;
    A.col(1) = c - a;
    A.col(2) = d - a;

    return A.determinant() / 6.;

}