// =====================================================================================================================
// Created by fabian on 4/30/25.
// mail@fabiankemper.de
// =====================================================================================================================

#pragma once
#ifndef VERTEX_SKINNER_H
#define VERTEX_SKINNER_H

// =====================================================================================================================

#include "vertex_predictor_onnx.h"
#include "volanim_utils/defs.h"

#include "../../volumetric_base/diffgeo/diffgeo.h"
#include "volumetric_base/layered_mesh.h"
#include "volumetric_base/state/filenames.h"
#include "volumetric_skinning/base/skinning_base.h"

// =====================================================================================================================

NAMESPACE_BEGIN(volumetric_skinning)
NAMESPACE_BEGIN(network)

// =====================================================================================================================

// =====================================================================================================================

class VertexNetworkSkinner : public volumetric_skinning::SkinningBase {
public:
  VertexNetworkSkinner();
  ~VertexNetworkSkinner();

  // new gender loaded (or initialize combinator)
  auto updated_mean_layers(volumetric_base::VolumetricState *state)
      -> void override;

  // new layers loaded for combination
  auto updated_input_layers(volumetric_base::VolumetricState *state)
      -> void override;

  // new pose set
  auto updated_pose(volumetric_base::VolumetricState *state) -> void override;

  // draw called
  auto perform_skinning(volumetric_base::VolumetricState *state,
                        volumetric_base::SkinnableMesh &skel,
                        volumetric_base::SkinnableMesh &musc,
                        volumetric_base::SkinnableMesh &skin, bool converge)
      -> void override;

  // only calculate blendshape
  auto perform_blendshape_skinning(volumetric_base::VolumetricState *state,
                                   volumetric_base::SkinnableMesh &skel,
                                   volumetric_base::SkinnableMesh &musc,
                                   volumetric_base::SkinnableMesh &skin)
      -> void override;

  // re-activate after cache
  auto wake_up() -> void override;

  // load a model
  auto load_model(const std::filesystem::path &model_filename) -> void;

  // imgui
  auto imgui_update(volumetric_base::VolumetricState *state) -> void override;

  auto predict(const Eigen::MatrixXd &rotations) -> void;

  // skinning methods
  auto skin(const volumetric_base::VolumetricState *state,
            const volumetric_base::LayeredMesh &mesh)
      -> volumetric_base::LayeredMesh;

  auto skin_blendshape(const volumetric_base::VolumetricState *state,
                       const volumetric_base::LayeredMesh &mesh_to_skin)
      -> volumetric_base::LayeredMesh;


    Eigen::VectorXf pca_coeffs;
    bool show_pca_mesh = false;

  // current model
  std::filesystem::path _model_path = std::filesystem::absolute(
      volanim::filenames::get_model_path("Vertex.zip"));
  bool show_correctives = false;
protected:
  volumetric_base::diffgeo::DifferentialGeometrySolver diffeo_solver;

  volumetric_base::LayeredMesh _mesh;
  volumetric_base::LayeredMesh _current_a_pose_mesh;
  onnx::VertexPredictor predictor;
  bool disable_correctives = false;
};

// =====================================================================================================================

NAMESPACE_END(network)
NAMESPACE_END(volumetric_skinning)

// =====================================================================================================================

#endif // VERTEX_SKINNER_H

// =====================================================================================================================
