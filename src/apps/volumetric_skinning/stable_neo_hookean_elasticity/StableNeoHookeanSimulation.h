//
// Created by gerrit on 5/26/25.
//
#pragma once

#include "volanim_utils/defs.h"

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <volumetric_base/layered_mesh.h>

#include "NeoHookeanEnergy.h"
#include "NeoHookeanEnergyQuadrature.h"
#include "StableNeoHookeanTets.h"
#include "volumetric_base/state/volumetric_state.h"
#include "volumetric_skinning/corotational_linear_elasticity/prism_based_corotational_elasticity.h"
#include "volumetric_skinning/thin_shell/bending_energy.h"
#include "volumetric_skinning/thin_shell/stretching_energy.h"



NAMESPACE_BEGIN(volumetric_skinning::neo_hookean)
    enum Discretization {
    Prism, Quadrature, Tets
};
class StableNeoHookeanSkinning {
    public:

        explicit StableNeoHookeanSkinning(volumetric_base::LayeredMesh &mesh);

        double compute_energy(const volumetric_base::LayeredMesh & layered_mesh);
        std::tuple<double, bool> do_step(const volumetric_base::LayeredMesh &deformed_mesh);
        std::vector<double> do_skinning_steps(const volumetric_base::LayeredMesh &deformed_mesh, int iterations);
        volumetric_base::LayeredMesh animate(const volumetric_base::VolumetricState &state);
        void animate(const volumetric_base::LayeredMesh &mesh, const volumetric_base::VolumetricState &state);
         Eigen::VectorXd elastic_gradient(const volumetric_base::LayeredMesh & deformed_mesh);
         Eigen::SparseMatrix<double> elastic_hessian(const volumetric_base::LayeredMesh & deformed_mesh, bool &need_update);

        bool patternAnalyzed = false;
        std::vector<int> constrained_mapping;
        float epsilon = 0;
        volumetric_base::LayeredMesh undeformed_mesh;
        double convergence_epsilon = 1e-5;
        thin_shells::ThinShellBendingEnergy bending_energy;
        thin_shells::StretchingEnergy stretching_energy;

        NeoHookeanEnergyQuadrature neo_hookean_energy_quadrature;
        NeoHookeanEnergy neo_hookean_energy;
        volumetric_skinning::neo_hookean::NeoHookeanEnergyTets neo_hookean_energy_tets;
        //Weights are smaller to account for elastic params in kPA
        float skin_bending_energy_weight =2e-6;
        float skin_stretching_energy_weight =3e-5;
        Discretization discretization = Prism;
        const int num_linesearch_steps  =12;
#ifdef USE_MKL
        Eigen::PardisoLDLT<longSparseMatrix> solver;
#else
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
#endif
    };




NAMESPACE_END(volumetric_skinning::neo_hookean)
