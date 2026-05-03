//
// Created by gerrit on 6/3/25.
//

#ifndef ENERGY_BASE_H
#define ENERGY_BASE_H
#include "volumetric_base/layered_mesh.h"
#include <Eigen/Sparse>
NAMESPACE_BEGIN(volumetric_skinning::newton)

class EnergyBase {
    public:
    virtual ~EnergyBase() = default;
    virtual void setup(volumetric_base::LayeredMesh &mesh, std::vector<int> constrained_mapping_);
    virtual double energy(const volumetric_base::LayeredMesh& deformed_mesh);
    virtual Eigen::SparseMatrix<double> hessian([[maybe_unused]] const volumetric_base::LayeredMesh &deformed_mesh, [[maybe_unused]] bool & needs_recomputation);
    virtual Eigen::VectorXd gradient(const volumetric_base::LayeredMesh& deformed_mesh);

    std::vector<int> constrained_mapping;
};
NAMESPACE_END(volumetric_skinning::newton)



#endif //ENERGY_BASE_H
