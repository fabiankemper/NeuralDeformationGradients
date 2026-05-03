#pragma once
#include "skinnable_mesh_renderer.h"
#include "volumetric_skinning/bary_warp/bary_warp.h"

NAMESPACE_BEGIN(volumetric_character)

class WarpedMeshRenderer : public SkinnableMeshRenderer
{
public:
  WarpedMeshRenderer(SkinnableMesh& highres_mesh, const std::vector<bary_warp::WarpWeights>& weights);
  ~WarpedMeshRenderer() override;
  void set_highres_topology(const pmp::SurfaceMesh &mesh);

  void draw_warped(const pmp::mat4& mv, const pmp::mat4& mvp, const pmp::mat3& nm,
                   const Eigen::MatrixXd& template_vertex_matrix,
                   const Eigen::MatrixXd& template_rest_vertex_matrix,
                   const std::string& draw_mode);


private:
  // Replacing SSBOs with TBOs and their Texture handles
  GLuint _weight_tbo = 0, _weight_tex = 0;
  GLuint _index_tbo = 0, _index_tex = 0;
  GLuint _template_tbo = 0, _template_tex = 0;
  GLuint _rest_template_tbo = 0, _rest_template_tex = 0;
  GLuint _highres_normal_tbo = 0, _highres_normal_tex = 0;

  std::vector<bary_warp::WarpWeights> _weights_data;
protected:
  void create_buffers() override;
  GLuint _orig_index_vbo = 0;
  static const char* warp_vshader;
};

NAMESPACE_END(volumetric_character)