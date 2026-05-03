#ifndef NEOHOOKEANENERGYTETS_H
#define NEOHOOKEANENERGYTETS_H
#include "../base/energy_base.h"
#include "../base/hessian_constructor.h"

namespace volumetric_skinning {
    namespace neo_hookean {

        class NeoHookeanEnergyTets : public newton::EnergyBase{
        public:
            Eigen::VectorXd gradient(const volumetric_base::LayeredMesh &deformed_mesh) override;
            void setup(volumetric_base::LayeredMesh &mesh, std::vector<int> constrained_mapping) override;
            Eigen::SparseMatrix<double> hessian(const volumetric_base::LayeredMesh &deformed_mesh, bool & need_update) override;
            double energy(const volumetric_base::LayeredMesh &deformed_mesh) override;
            [[nodiscard]] std::array<double, 3> get_material_parameters(int i) const;
            //Lamè Parameters, derived from E=15/45kPA, Poisson's ratio =0.48
            float skin_lame_lambda = 81;
            float skin_lame_mu = 5 ;
            float muscle_lame_lambda = 240;
            float muscle_lame_mu =  15;
            volumetric_base::LayeredMesh undeformed_mesh;
            Eigen::SparseMatrix<double> gradient_;
            bool hessian_needs_only_update=false;
            std::vector<std::tuple<int,int,int,int>> tets;
            std::vector<double> weights;
            std::vector<Eigen::Matrix<double,3,4>> gradients;
            int num_dofs=0;
            newton::HessianConstructor<4> hessian_constructor;
        };

    } // neo_hookean
} // volumetric_skinning

#endif //NEOHOOKEANENERGY_H
