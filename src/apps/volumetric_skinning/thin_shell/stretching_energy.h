//
// Created by gerrit on 6/4/25.
//

#ifndef STRETCHING_ENERGY_H
#define STRETCHING_ENERGY_H
#include "../base/energy_base.h"
#include "../base/hessian_constructor.h"

NAMESPACE_BEGIN(volumetric_skinning::thin_shells)
    class StretchingEnergy : public newton::EnergyBase {

    public:
        StretchingEnergy() = default;
        ~StretchingEnergy() override = default;
        void setup(volumetric_base::LayeredMesh &mesh, std::vector<int> constrained_mapping) override;
        double energy(const volumetric_base::LayeredMesh &deformed_mesh) override;
        Eigen::VectorXd gradient(const volumetric_base::LayeredMesh &deformed_mesh) override;
        Eigen::SparseMatrix<double> hessian(const volumetric_base::LayeredMesh &deformed_mesh, bool &patternAnalyzed) override;

    private:
        std::vector<Eigen::Matrix<double,6,6>> local_hessians;
        std::vector<double> weights;
        std::vector<std::array<int,2> > edges;
        std::vector<double> original_lengths;
        std::vector<int> constrained_mapping;
        int n_active_vertices = 0;
        newton::HessianConstructor<2> hessian_constructor;

};

NAMESPACE_END(volumetric_skinning::thin_shells)

#endif //STRETCHING_ENERGY_H
