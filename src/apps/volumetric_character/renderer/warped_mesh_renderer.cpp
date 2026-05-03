#include "warped_mesh_renderer.h"
#include "shader/warped_mesh_shader.h"

#include <glad/gl.h>

NAMESPACE_BEGIN(volumetric_character)

WarpedMeshRenderer::WarpedMeshRenderer(SkinnableMesh& mesh, const std::vector<bary_warp::WarpWeights>& weights)
    : SkinnableMeshRenderer(mesh), _weights_data(weights)
{
    _phong_shader.source(volumetric_base::shaders::warp_vshader, volumetric_base::shaders::phong_fshader_normal_debug);
}

WarpedMeshRenderer::~WarpedMeshRenderer()
{
    glDeleteBuffers(1, &_weight_tbo);
    glDeleteTextures(1, &_weight_tex);
    glDeleteBuffers(1, &_index_tbo);
    glDeleteTextures(1, &_index_tex);
    glDeleteBuffers(1, &_template_tbo);
    glDeleteTextures(1, &_template_tex);
    glDeleteBuffers(1, &_rest_template_tbo);
    glDeleteTextures(1, &_rest_template_tex);
    glDeleteBuffers(1, &_highres_normal_tbo);
    glDeleteTextures(1, &_highres_normal_tex);
}

void WarpedMeshRenderer::set_highres_topology(const pmp::SurfaceMesh& mesh_)
{
    pmp::SurfaceMesh mesh = mesh_;
    pmp::vertex_normals(mesh);
    if (!_vertex_array_object) create_buffers();

    // 1. Upload Indices (Keep this exactly the same)
    std::vector<int> orig_indices;
    orig_indices.reserve(mesh.n_faces() * 3);
    for (auto f: mesh.faces()) {
        for (auto v: mesh.vertices(f)) {
            orig_indices.push_back(static_cast<int>(v.idx()));
        }
    }
    glBindVertexArray(_vertex_array_object);
    if (_orig_index_vbo == 0)
        glGenBuffers(1, &_orig_index_vbo);
    glBindBuffer(GL_ARRAY_BUFFER, _orig_index_vbo);
    glBufferData(GL_ARRAY_BUFFER, orig_indices.size() * sizeof(int), orig_indices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(4);
    glVertexAttribIPointer(4, 1, GL_INT, 0, nullptr);
    glBindVertexArray(0);

    _vertexBufferSize = static_cast<GLsizei>(orig_indices.size());
    auto v_normals = mesh.get_vertex_property<pmp::vec3>("v:normal");
    std::vector<pmp::vec4> unique_normals(mesh.n_vertices(), pmp::vec4(0.0f, 1.0f, 0.0f, 0.0f));
    for (auto v: mesh.vertices()) {
        unique_normals[v.idx()] = pmp::vec4(v_normals[v][0], v_normals[v][1], v_normals[v][2], 0.0f);
    }

    if (_highres_normal_tbo == 0) {
        glGenBuffers(1, &_highres_normal_tbo);
        glGenTextures(1, &_highres_normal_tex);
    }

    glBindBuffer(GL_TEXTURE_BUFFER, _highres_normal_tbo);
    glBufferData(GL_TEXTURE_BUFFER, unique_normals.size() * sizeof(pmp::vec4), unique_normals.data(), GL_STATIC_DRAW);

    glBindTexture(GL_TEXTURE_BUFFER, _highres_normal_tex);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, _highres_normal_tbo);
}

void WarpedMeshRenderer::create_buffers()
{
    SkinnableMeshRenderer::create_buffers();

    // Flatten structures for easier TBO reading
    std::vector<float> flat_weights;
    std::vector<int32_t> flat_indices;
    flat_weights.reserve(_weights_data.size() * 32);
    flat_indices.reserve(_weights_data.size() * 32);

    // copy weights
    for (const auto& w: _weights_data) {
        // float sum = 0.0f;
        for (int i = 0; i < 32; ++i) {
            flat_indices.push_back(static_cast<int32_t>(w.indices[i]));

            float weight = static_cast<float>(w.weights[i]);
            if (weight < 0.0F) {
                weight = 0.0F;
            }
            flat_weights.push_back(weight);
            // sum += weight;
        }
        // if (abs(sum - 1.0f) > 1.0e-4) {
        //     std::cerr << "Weight sum = " << sum << std::endl;
        // }
    }

    glGenBuffers(1, &_weight_tbo);
    glGenTextures(1, &_weight_tex);
    glBindBuffer(GL_TEXTURE_BUFFER, _weight_tbo);
    glBufferData(GL_TEXTURE_BUFFER, flat_weights.size() * sizeof(float), flat_weights.data(), GL_STATIC_DRAW);
    glBindTexture(GL_TEXTURE_BUFFER, _weight_tex);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_R32F, _weight_tbo);

    glGenBuffers(1, &_index_tbo);
    glGenTextures(1, &_index_tex);
    glBindBuffer(GL_TEXTURE_BUFFER, _index_tbo);
    glBufferData(GL_TEXTURE_BUFFER, flat_indices.size() * sizeof(int32_t), flat_indices.data(), GL_STATIC_DRAW);
    glBindTexture(GL_TEXTURE_BUFFER, _index_tex);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_R32I, _index_tbo);

    glGenBuffers(1, &_template_tbo);
    glGenTextures(1, &_template_tex);
    glGenBuffers(1, &_rest_template_tbo);
    glGenTextures(1, &_rest_template_tex);
}

void WarpedMeshRenderer::draw_warped(
    const pmp::mat4& mv, const pmp::mat4& mvp, const pmp::mat3& nm,
    const Eigen::MatrixXd& template_vertex_matrix,
    const Eigen::MatrixXd& template_rest_vertex_matrix,
    const std::string& draw_mode)
{
    if (_vertex_array_object == 0U) {
        create_buffers();
    }

    std::vector<pmp::vec4> float_pos;
    float_pos.reserve(template_vertex_matrix.rows());
    for (int i = 0; i < template_vertex_matrix.rows(); ++i) {
        float_pos.emplace_back(template_vertex_matrix(i, 0),
                               template_vertex_matrix(i, 1),
                               template_vertex_matrix(i, 2), 1.0f);
    }

    std::vector<pmp::vec4> float_rest_pos;
    float_rest_pos.reserve(template_rest_vertex_matrix.rows());
    for (int i = 0; i < template_rest_vertex_matrix.rows(); ++i) {
        float_rest_pos.emplace_back(template_rest_vertex_matrix(i, 0),
                                    template_rest_vertex_matrix(i, 1),
                                    template_rest_vertex_matrix(i, 2), 1.0f);
    }

    glBindBuffer(GL_TEXTURE_BUFFER, _template_tbo);
    glBufferData(GL_TEXTURE_BUFFER, float_pos.size() * sizeof(pmp::vec4), float_pos.data(), GL_STREAM_DRAW);
    glBindTexture(GL_TEXTURE_BUFFER, _template_tex);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, _template_tbo);

    glBindBuffer(GL_TEXTURE_BUFFER, _rest_template_tbo);
    glBufferData(GL_TEXTURE_BUFFER, float_rest_pos.size() * sizeof(pmp::vec4), float_rest_pos.data(), GL_STREAM_DRAW);
    glBindTexture(GL_TEXTURE_BUFFER, _rest_template_tex);
    glTexBuffer(GL_TEXTURE_BUFFER, GL_RGBA32F, _rest_template_tbo);

    // Pre-activate shader so we can set up the TBO samplers cleanly
    _phong_shader.use();

    // Start at Texture Unit 1 to ensure we don't clobber GL_TEXTURE0 which base classes might use
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_BUFFER, _template_tex);
    _phong_shader.set_uniform("templateBuf", 1);

    glActiveTexture(GL_TEXTURE2);
    glBindTexture(GL_TEXTURE_BUFFER, _rest_template_tex);
    _phong_shader.set_uniform("restTemplateBuf", 2);

    glActiveTexture(GL_TEXTURE3);
    glBindTexture(GL_TEXTURE_BUFFER, _index_tex);
    _phong_shader.set_uniform("indexBuf", 3);

    glActiveTexture(GL_TEXTURE4);
    glBindTexture(GL_TEXTURE_BUFFER, _weight_tex);
    _phong_shader.set_uniform("weightBuf", 4);

    glActiveTexture(GL_TEXTURE5);
    glBindTexture(GL_TEXTURE_BUFFER, _highres_normal_tex);
    _phong_shader.set_uniform("highresNormalBuf", 5);

    // Reset back to standard texture unit for the rest of the drawing state
    glActiveTexture(GL_TEXTURE0);

    // Pass down to the standard SkinnableMeshRenderer
    this->draw(mv, mvp, nm, draw_mode);
}

NAMESPACE_END(volumetric_character)