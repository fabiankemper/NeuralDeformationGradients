//
// Created by gerrit on 6/26/25.
//

#include "StableNeoHookeanTets.h"

#include "neo_hookean_derivatives.h"
#include "volumetric_base/diffgeo/prism_diffgeo.h"
#include "volumetric_base/utility/matrix_util.hpp"

template <int n>
Eigen::Matrix<double, 9, n * 3> expand_gradient(const Eigen::Matrix<double, 3, n>& localGrad) {
    return matrix_util::vectorize_gradient(localGrad);
}

namespace volumetric_skinning {
    namespace neo_hookean {

    std::array<double, 3> NeoHookeanEnergyTets::get_material_parameters(const int i) const {
        double mu, lambda;
        if (i%2 ==0) {
            mu = muscle_lame_mu;
            lambda = muscle_lame_lambda;
        }
        else {
            mu = skin_lame_mu;
            lambda = skin_lame_lambda;
        }
        return {mu,lambda, 1.0+mu/lambda};
    }

double NeoHookeanEnergyTets::energy(
    const volumetric_base::LayeredMesh &deformed_mesh) {
    Eigen::MatrixXd defgrads = gradient_*deformed_mesh.toVertexMatrix();
    double energy =0;
#pragma omp parallel for reduction(+:energy)
    for (int i=0; i< defgrads.rows()/3;i++) {
        const Eigen::Matrix3d F = defgrads.block<3,3>(3*i,0);
        auto [mu, lambda, alpha] = get_material_parameters(i);
        energy += weights[i]*stable_neo_hookean::compute_energy(F,mu,lambda,alpha);
    }
    return energy;
}

Eigen::VectorXd NeoHookeanEnergyTets::gradient(
    const volumetric_base::LayeredMesh &deformed_mesh) {
    Eigen::VectorXd global_gradient(num_dofs*3);
    global_gradient.setZero();
    Eigen::MatrixXd defgrads = gradient_ * deformed_mesh.toVertexMatrix();
    Eigen::MatrixXd positions = deformed_mesh.toVertexMatrix();

    for (unsigned int i=0; i< tets.size(); i++) {
        const Eigen::Matrix3d F = defgrads.block<3,3>(i*3,0);
        const Eigen::Matrix<double,3,4> local_gradient = gradients[i];
        auto [mu, lambda, alpha] = get_material_parameters(i);
        auto [i1,i2,i3,i4] = tets[i];
        const Eigen::Matrix<double,3,4> grad = weights[i]* stable_neo_hookean::gradient(F,mu,lambda,alpha).transpose()*local_gradient;


        if (constrained_mapping[i1] !=-1)
            global_gradient.block<3,1>(constrained_mapping[i1]*3,0) += grad.col(0);
        if (constrained_mapping[i2] !=-1)
            global_gradient.block<3,1>(constrained_mapping[i2]*3,0) += grad.col(1);
        if (constrained_mapping[i3] !=-1)
            global_gradient.block<3,1>(constrained_mapping[i3]*3,0) += grad.col(2);
        if (constrained_mapping[i4] !=-1)
            global_gradient.block<3,1>(constrained_mapping[i4]*3,0) += grad.col(3);
    }

    return global_gradient;
}

        Eigen::Matrix<double,3,4> getTetGrad(const volumetric_base::Tetrahedron& tet, const Eigen::MatrixXd &posis) {
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


        double getTetVolume(const volumetric_base::Tetrahedron& tet, const Eigen::MatrixXd &posis) {
        auto [i1,i2,i3,i4] = tet;

        Eigen::Matrix<double,3,3> edges;
        edges.row(0) = posis.row(i1) - posis.row(i4);
        edges.row(1) = posis.row(i2) - posis.row(i4);
        edges.row(2) = posis.row(i3) - posis.row(i4);

        return edges.determinant()/6.0;
    }

void NeoHookeanEnergyTets::setup(volumetric_base::LayeredMesh &mesh, std::vector<int> constrained_mapping) {
        gradient_ = volumetric_base::tetrahedral::getGradientMatrix(mesh);
        undeformed_mesh = mesh;
        Eigen::MatrixXd posis = mesh.toVertexMatrix();
        for (const auto &tet: volumetric_base::tetrahedral::getTetrahedra(mesh)) {
            tets.emplace_back(tet);
            gradients.emplace_back(getTetGrad(tet, posis));
            weights.emplace_back(std::abs(getTetVolume(tet,posis)));
        }
        this->constrained_mapping = constrained_mapping;
        num_dofs =0;
        for (int idx : constrained_mapping) {
            if (idx != -1)
                num_dofs++;
        }
        hessian_constructor = newton::HessianConstructor<4>(constrained_mapping);
    }

Eigen::SparseMatrix<double> NeoHookeanEnergyTets::hessian(
    const volumetric_base::LayeredMesh &deformed_mesh, [[maybe_unused]] bool & patternAnalyzed) {

    Eigen::MatrixXd defgrads = gradient_ * deformed_mesh.toVertexMatrix();
    Eigen::MatrixXd positions = deformed_mesh.toVertexMatrix();
        auto local_hess = [&](std::array<int,4> tet, int i)->Eigen::Matrix<double,12,12> {
            auto [i1,i2,i3,i4] = tet;
            const Eigen::Matrix3d F = defgrads.block<3,3>(3*i,0);
            const Eigen::Matrix<double,9,12> G = expand_gradient(gradients[i]);

            Eigen::MatrixXd posis(4,3);
            posis.row(0) = positions.row(i1);
            posis.row(1) = positions.row(i2);
            posis.row(2) = positions.row(i3);
            posis.row(3) = positions.row(i4);


            const auto [mu, lambda, alpha] = get_material_parameters(static_cast<int>(i));
            // G^T * d^2 PSI(X) / dF^2 * G
            return weights[i]* G.transpose()*matrix_util::psd_projection_to_abs(stable_neo_hookean::hessian(F,mu,lambda,alpha))*G;

        };
        std::vector<std::array<int,4>> indices;
        for (auto tet: volumetric_base::tetrahedral::getTetrahedra(undeformed_mesh)) {
            auto [i1,i2,i3,i4] = tet;
            std::array ids = {i1,i2,i3,i4};
            indices.emplace_back(ids);
        }
        return hessian_constructor.assemble_hessian(local_hess,  indices);
}


}

}
