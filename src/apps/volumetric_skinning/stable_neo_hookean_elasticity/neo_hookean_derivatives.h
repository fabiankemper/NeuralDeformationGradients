//
// Created by gerrit on 5/26/25.
//

#ifndef NEO_HOOKEAN_DERIVATIVES_H
#define NEO_HOOKEAN_DERIVATIVES_H
#include <Eigen/Core>

#include "apps/volumetric_character/skinning/tet_volumetric.h"
#include "invariants_and_derivatives.h"
#include "volanim_utils/defs.h"
#include "volumetric_base/utility/matrix_util.hpp"

NAMESPACE_BEGIN(volumetric_skinning::stable_neo_hookean)

/**
 *  Implementation of the Smith et al. (2019) stable neo-hookean energy and its derivatives.
 *  Calculations are based on the Jacobians and Hessians of the Elastic Invariants (see
 *  elastic_invariants/invariants_and_derivatives.hpp
 */


inline double compute_energy(const Eigen::Matrix3d &F, double mu, double lambda, double alpha) {
    const double Ic = (F.transpose()*F).trace();
    const double J = F.determinant();
    return (mu/2)*(Ic-3)+(lambda/2)*(J-alpha)*(J-alpha);
}

inline Eigen::Matrix3d gradient(const Eigen::Matrix3d & F, double mu, double lambda, double alpha) {
    const double J = F.determinant();
    const Eigen::Matrix3d Finvt= F.inverse().transpose();
    return mu*F+lambda*J*J*Finvt-lambda*alpha*J*Finvt;

}

inline Eigen::Matrix<double,9,9> hessian(const Eigen::Matrix3d& F, double mu, double lambda, double alpha) {
    double i3 = elastic_invariants::I3(F);

    double dpsi_di2 = (1.0)*(mu/2);

    double dpsi_di3 = lambda*(i3-alpha);


    double dpsi_di2_squared =0;

    double dpsi_di3_squared = lambda;



    Eigen::Vector<double,9> g2 = elastic_invariants::I2_grad(F);
    Eigen::Vector<double,9> g3 = elastic_invariants::I3_grad(F);

    const Eigen::Matrix<double,9,9> H2 = elastic_invariants::I2_hess(F);
    const Eigen::Matrix<double,9,9> H3 = elastic_invariants::I3_hess(F);

    return dpsi_di2*H2 + dpsi_di3*H3
        + dpsi_di2_squared*(g2*g2.transpose())
        + dpsi_di3_squared*(g3*g3.transpose());
}

NAMESPACE_END(volumetric_skinning::stable_neo_hookean)

#endif //NEO_HOOKEAN_DERIVATIVES_H