//
// Created by gerrit on 6/3/25.
//

#ifndef BENDING_ENERGY_H
#define BENDING_ENERGY_H
#include <array>
#include <vector>

#include "../base/energy_base.h"
#include "../base/hessian_constructor.h"
#include "apps/volumetric_base/layered_mesh.h"

NAMESPACE_BEGIN(volumetric_skinning::thin_shells)
    class ThinShellBendingEnergy : public volumetric_skinning::newton::EnergyBase{

    public:
        ~ThinShellBendingEnergy() override = default;
        ThinShellBendingEnergy() = default;

        auto setup(volumetric_base::LayeredMesh &mesh, std::vector<int> constrained_mapping_) -> void override;
        double energy(const volumetric_base::LayeredMesh &deformed_mesh) override;
        Eigen::VectorXd gradient(const volumetric_base::LayeredMesh &deformed_mesh) override;
        Eigen::SparseMatrix<double> hessian(const volumetric_base::LayeredMesh &deformed_mesh, bool &patternAnalyzed) override;


        private:
            /** Diamonds stored as i,j,k,l
            //         k
            //        / \
            //       /   \
            //      /     \
            //     i-------j
            //      \     /
            //       \   /
            //        \ /
            //         l
            **/
        std::vector<std::array<int, 4> > diamonds;
        std::vector<double> weights;
        std::vector<double> original_angles;
        std::vector<int> constrained_mapping;
        std::vector<double*> sparse_mat_entries;
        Eigen::SparseMatrix<double> hessian_;
        std::vector<Eigen::Matrix<double,12,12>> local_hessians;
        int n_active_vertices=0;
            newton::HessianConstructor<4> hessian_constructor;
};

NAMESPACE_END(volumetric_skinning::thin_shell)

#endif //BENDING_ENERGY_H
