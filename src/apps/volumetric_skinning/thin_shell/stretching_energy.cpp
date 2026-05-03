#include "stretching_energy.h"
#include "stretching_derivatives.h"
#include "volumetric_skinning/neural_skinning/defgrad_skinner.h"

void volumetric_skinning::thin_shells::StretchingEnergy::setup(volumetric_base::LayeredMesh &mesh,
    const std::vector<int> constrained_mapping_) {
    this->constrained_mapping = constrained_mapping_;
    n_active_vertices = 0;
    for (int idx : constrained_mapping) {
        if (idx != -1) n_active_vertices++;
    }
    const Eigen::MatrixXd posis = mesh.toVertexMatrix();
    for (pmp::Edge e: mesh.edges()) {
        unsigned int i = volumetric_base::LayeredMesh::Index(mesh.vertex(e,0), volumetric_base::Layer::SKIN);
        unsigned int j = volumetric_base::LayeredMesh::Index(mesh.vertex(e,1), volumetric_base::Layer::SKIN);
        const Eigen::Vector3d Pi = posis.row(i);
        const Eigen::Vector3d Pj = posis.row(j);
        double length = (Pi-Pj).norm();
        original_lengths.emplace_back(length);
        std::array edge= {static_cast<int>(i),static_cast<int>(j)};
        edges.emplace_back(edge);
        weights.emplace_back((length*length+1e-6)/2);
    }
    hessian_constructor = newton::HessianConstructor<2>(constrained_mapping);

}

double volumetric_skinning::thin_shells::StretchingEnergy::energy(const volumetric_base::LayeredMesh &deformed_mesh) {
    double energy = 0.0;
    const Eigen::MatrixXd posis = deformed_mesh.toVertexMatrix();
#pragma omp parallel for reduction(+:energy)
    for (int edge_idx=0; edge_idx < static_cast<int>(edges.size()); edge_idx++) {
        auto [i,j] = edges[edge_idx];
        const Eigen::Vector3d Pi = posis.row(i);
        const Eigen::Vector3d Pj = posis.row(j);
        energy += weights[edge_idx]*stretching_energy(Pi, Pj, original_lengths[edge_idx]);
    }

    return energy;

}

Eigen::VectorXd volumetric_skinning::thin_shells::StretchingEnergy::gradient(
    const volumetric_base::LayeredMesh &deformed_mesh) {
    Eigen::VectorXd gradient(n_active_vertices*3);
    gradient.setZero();
    const Eigen::MatrixXd posis = deformed_mesh.toVertexMatrix();
    for (unsigned int edge_idx=0; edge_idx<edges.size(); edge_idx++) {
        const auto [i,j] = edges[edge_idx];
        const double weight = weights[edge_idx];
        const Eigen::Vector3d Pi = posis.row(i);
        const Eigen::Vector3d Pj = posis.row(j);
        const Eigen::Vector<double,6> local_grad = weight*stretching_gradient(Pi,Pj, original_lengths[edge_idx]);
        if (constrained_mapping[i] != -1) {
            gradient.segment<3>(constrained_mapping[i]*3) += local_grad.segment<3>(0);
        }
        if (constrained_mapping[j] != -1) {
            gradient.segment<3>(constrained_mapping[j]*3) += local_grad.segment<3>(3);
        }
    }
    return gradient;
}

Eigen::SparseMatrix<double> volumetric_skinning::thin_shells::StretchingEnergy::hessian(
    const volumetric_base::LayeredMesh &deformed_mesh, [[maybe_unused]] bool &patternAnalyzed) {
    const Eigen::MatrixXd posis = deformed_mesh.toVertexMatrix();

    auto construct_local_hess = [&](std::array<int,2> edge, int edge_idx)->Eigen::Matrix<double,6,6> {
        const auto [i,j] = edge;
        const Eigen::Vector3d Pi = posis.row(i);
        const Eigen::Vector3d Pj = posis.row(j);
        const double weight = weights[edge_idx];
        const Eigen::Matrix<double,6,6> local_hess = weight*stretching_hessian(Pi-Pj,original_lengths[edge_idx]);
        return matrix_util::psd_projection_to_zero(local_hess);
    };

    return hessian_constructor.assemble_hessian(construct_local_hess, edges);

}
