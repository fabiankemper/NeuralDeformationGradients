//
// Created by gerrit on 6/3/25.
//

#include "energy_base.h"

auto volumetric_skinning::newton::EnergyBase::setup([[maybe_unused]] volumetric_base::LayeredMesh &mesh,
                                                    [[maybe_unused]] std::vector<int> constrained_mapping_) -> void {
    std::cout << "Energy did not define setup" << std::endl;
    exit(1);

}

double volumetric_skinning::newton::EnergyBase::energy([[maybe_unused]] const volumetric_base::LayeredMesh &deformed_mesh) {
    std::cout << "Energy did not define energy" << std::endl;
    exit(1);
}

Eigen::SparseMatrix<double> volumetric_skinning::newton::EnergyBase::hessian([[maybe_unused]] const volumetric_base::LayeredMesh &deformed_mesh, [[maybe_unused]] bool & needs_recomputation) {
    std::cout << "Energy did not define hessian" << std::endl;
    exit(1);
}

Eigen::VectorXd volumetric_skinning::newton::EnergyBase::gradient([[maybe_unused]] const volumetric_base::LayeredMesh &deformed_mesh) {
    std::cout << "Energy did not define gradient" << std::endl;
    exit(1);
}
