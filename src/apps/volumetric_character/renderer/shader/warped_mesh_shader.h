#pragma once

namespace volumetric_base::shaders
{

inline const auto* const warp_vshader = R"glsl(
#version 330 core

layout (location=0) in vec3 v_position;
layout (location=1) in vec3 v_normal;
layout (location=2) in vec2 v_tex;
layout (location=3) in vec3 v_color;
layout (location=4) in int  v_orig_index;

uniform mat4 modelview_projection_matrix;
uniform mat4 modelview_matrix;
uniform mat3 normal_matrix;
uniform float point_size;

// Replaced SSBOs with TBOs (Texture Buffer Objects) for GL 3.3 support
uniform samplerBuffer templateBuf;       // GL_RGBA32F
uniform samplerBuffer restTemplateBuf;   // GL_RGBA32F
uniform isamplerBuffer indexBuf;         // GL_R32I (Int lookup)
uniform samplerBuffer weightBuf;         // GL_R32F (Float lookup)
uniform samplerBuffer highresNormalBuf;  // GL_RGBA32F

out vec3 v2f_normal;
out vec2 v2f_tex;
out vec3 v2f_view;
out vec3 v2f_color;

void main() {
    vec3 p_bar = vec3(0.0);
    vec3 q_bar = vec3(0.0);

    // Calculate the base offset for this vertex (flattened 2D array)
    int base_idx = v_orig_index * 32;

    // Calculate Centroids
    for(int i=0; i<32; ++i) {
        float weight = texelFetch(weightBuf, base_idx + i).x;
        if (weight <= 0.0) continue;

        int idx = texelFetch(indexBuf, base_idx + i).x;
        p_bar += weight * texelFetch(restTemplateBuf, idx).xyz;
        q_bar += weight * texelFetch(templateBuf, idx).xyz;
    }

    mat3 A = mat3(0.0);
    mat3 B = mat3(0.0);

    // Calculate Covariance Matrices
    for(int i=0; i<32; ++i) {
        float weight = texelFetch(weightBuf, base_idx + i).x;
        if (weight <= 0.0) continue;

        int idx = texelFetch(indexBuf, base_idx + i).x;
        vec3 p_hat = texelFetch(restTemplateBuf, idx).xyz - p_bar;
        vec3 q_hat = texelFetch(templateBuf, idx).xyz - q_bar;

        A += weight * outerProduct(q_hat, p_hat);
        B += weight * outerProduct(p_hat, p_hat);
    }

    // Diagonal Loading
    float epsilon = 1e-9;
    B[0][0] += epsilon;
    B[1][1] += epsilon;
    B[2][2] += epsilon;

    float det_B = determinant(B);
    mat3 M;

    if (abs(det_B) < 1e-10) {
        M = mat3(1.0);
    } else {
        M = A * inverse(B);
    }

    // Transform position
    vec4 pos = vec4(q_bar, 1.0);

    // Normal Transformation
    mat3 M_inv_T = transpose(inverse(M));
    vec3 n_rest = texelFetch(highresNormalBuf, v_orig_index).xyz;

    if (any(isnan(n_rest)) || length(n_rest) < 0.001) n_rest = vec3(0.0, 1.0, 0.0);

    v2f_normal   = normal_matrix * (M_inv_T * n_rest);
    v2f_view     = -(modelview_matrix * pos).xyz;
    v2f_tex      = v_tex;
    v2f_color    = v_color;

    gl_PointSize = point_size;
    gl_Position  = modelview_projection_matrix * pos;
}
)glsl";

inline auto* const phong_fshader_normal_debug =
R"glsl(
#version 330 core
precision highp float;

in vec3  v2f_normal;
in vec2  v2f_tex;
in vec3  v2f_view;
in vec3  v2f_color;

uniform bool   use_lighting;
uniform bool   use_texture;
uniform bool   use_srgb;
uniform bool   use_vertex_color;
uniform vec3   front_color;
uniform vec3   back_color;
uniform float  ambient;
uniform float  diffuse;
uniform float  specular;
uniform float  shininess;
uniform float  alpha;
uniform vec3   light1;
uniform vec3   light2;

uniform sampler2D mytexture;

out vec4 f_color;

void main()
{
    vec3 base_color = use_vertex_color ? v2f_color : (gl_FrontFacing ? front_color : back_color);
    if (use_texture) {
        base_color *= texture(mytexture, v2f_tex).xyz;
    }

    vec3 final_color = base_color;

    if (use_lighting)
    {
        vec3 N = normalize(gl_FrontFacing ? v2f_normal : -v2f_normal);
        vec3 L1 = normalize(light1);
        vec3 L2 = normalize(light2);
        vec3 V = normalize(v2f_view);

        float wrap1 = dot(N, L1) * 0.5 + 0.5;
        float wrap2 = dot(N, L2) * 0.5 + 0.5;

        float smooth_wrap1 = smoothstep(0.1, 0.9, wrap1);

        vec3 main_light = base_color * diffuse * smooth_wrap1;
        vec3 fill_light = base_color * ambient * (wrap2 * 0.5 + 0.5);

        vec3 R1 = reflect(-L1, N);
        float spec_angle1 = max(dot(V, R1), 0.0);
        float spec_intensity1 = pow(spec_angle1, shininess);

        vec3 R2 = reflect(-L2, N);
        float spec_angle2 = max(dot(V, R2), 0.0);
        float spec_intensity2 = pow(spec_angle2, shininess);

        vec3 specular_light = vec3(1.0) * specular * (spec_intensity1 + spec_intensity2);

        final_color = main_light + fill_light + specular_light;
    }

    if (use_srgb) final_color = pow(clamp(final_color, 0.0, 1.0), vec3(0.45));

    f_color = vec4(final_color, alpha);
}
)glsl";

} // namespace volumetric_base::shaders