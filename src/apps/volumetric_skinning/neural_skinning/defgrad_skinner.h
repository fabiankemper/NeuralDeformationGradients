// =====================================================================================================================
// Created by fabian on 4/29/25.
// mail@fabiankemper.de
// =====================================================================================================================

#pragma once
#ifndef DEFGRAD_SKINNER_H
#define DEFGRAD_SKINNER_H

// =====================================================================================================================

#include "defgrad_predictor_onnx.h"
#include "volanim_utils/defs.h"

#include "../../volumetric_base/diffgeo/diffgeo.h"

#include "volumetric_skinning/base/skinning_base.h"


// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_skinning)
NAMESPACE_BEGIN(network)
// =====================================================================================================================

class DefgradNetworkSkinner : public SkinningBase
{
public:
    DefgradNetworkSkinner();

    ~DefgradNetworkSkinner() override;

    // new gender loaded (or initialize combinator)
    auto updated_mean_layers(volumetric_base::VolumetricState* state)
        -> void override;

    // new layers loaded for combination
    auto updated_input_layers(volumetric_base::VolumetricState* state)
        -> void override;

    // new pose set
    auto updated_pose(volumetric_base::VolumetricState* state) -> void override;

    // get network predicts
    auto update_prediction(const Eigen::MatrixXd& rotations)
        -> void;

    // draw called
    auto perform_skinning(volumetric_base::VolumetricState* state,
                          volumetric_base::SkinnableMesh& skel,
                          volumetric_base::SkinnableMesh& musc,
                          volumetric_base::SkinnableMesh& skin, bool converge)
        -> void override;

    auto predict_deformation_gradients(volumetric_base::VolumetricState* state)
        -> Eigen::MatrixXd;

    // re-activate after cache
    auto wake_up() -> void override;

    // load a model
    auto load_model(const std::filesystem::path& model_filename) -> void;

    // imgui
    auto imgui_update(volumetric_base::VolumetricState* state) -> void override;

    // perform skinning
    auto skin(const volumetric_base::VolumetricState* state,
              const volumetric_base::LayeredMesh& mesh)
        -> volumetric_base::LayeredMesh;

    auto skin_no_rot(volumetric_base::VolumetricState* state,
                     const volumetric_base::LayeredMesh& mesh)
        -> volumetric_base::LayeredMesh;

    auto skin_pose_corrective(const volumetric_base::VolumetricState* state)
        -> volumetric_base::LayeredMesh;

    std::filesystem::path model_path = std::filesystem::absolute(
        volanim::filenames::get_model_path("Defgrad.zip"));

protected:
    DefGradPredictor predictor;
    volumetric_base::LayeredMesh mesh_;
    volumetric_base::LayeredMesh animated_mesh_;
    Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> defgrads_;
    volumetric_base::diffgeo::DifferentialGeometrySolver* diffgeo_solver_;
    Eigen::VectorXf pca_coeffs_;
    bool show_pca_mesh_ = false;
    Eigen::MatrixXd rotations_;
    bool show_pose_correctives_ = false;
    bool disable_network_predictions_ = false;
};

// =====================================================================================================================

NAMESPACE_END(network)

NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================

#endif // DEFGRAD_SKINNER_H

// =====================================================================================================================
