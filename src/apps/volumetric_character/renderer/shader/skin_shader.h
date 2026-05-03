#pragma once

namespace volumetric_base::shaders {

static auto* const phong_vshader =
#ifndef __EMSCRIPTEN__
    "#version 330 core\n"
#else
    "#version 300 es\n"
#endif
    R"glsl(
layout (location=0) in vec4 v_position;
layout (location=1) in vec3 v_normal;
layout (location=2) in vec2 v_tex;
layout (location=3) in vec3 v_color;

out vec3 v2f_normal;
out vec2 v2f_tex;
out vec3 v2f_view;
out vec3 v2f_color;

uniform mat4 modelview_projection_matrix;
uniform mat4 modelview_matrix;
uniform mat3 normal_matrix;
uniform float point_size;
uniform bool show_texture_layout;

void main()
{
    v2f_normal   = normal_matrix * v_normal;
    v2f_tex      = v_tex;
    vec4 pos     = show_texture_layout ? vec4(v_tex, 0.0, 1.0) : v_position;
    v2f_view     = -(modelview_matrix * pos).xyz;
    v2f_color    = v_color;
    gl_PointSize = point_size;
    gl_Position  = modelview_projection_matrix * pos;
}
)glsl";

static auto* const phong_fshader =
#ifndef __EMSCRIPTEN__
    "#version 330 core\n"
#else
    "#version 300 es\n"
#endif
R"glsl(
precision mediump float;

in vec3  v2f_normal;
in vec2  v2f_tex;
in vec3  v2f_view;
in vec3  v2f_color;

uniform bool   use_lighting;
uniform bool   use_texture;
uniform bool   use_srgb;
uniform bool   use_vertex_color;
uniform bool   use_round_points;
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
        // normalize vectors
        vec3 N = normalize(gl_FrontFacing ? v2f_normal : -v2f_normal);
        vec3 L1 = normalize(light1);
        vec3 L2 = normalize(light2);
        vec3 V = normalize(v2f_view);

        // --- Diffuse ---
        // This wraps the light 50% around the dark side, preventing harsh black shadows
        float wrap1 = dot(N, L1) * 0.5 + 0.5;
        float wrap2 = dot(N, L2) * 0.5 + 0.5;

        // Smoothstep to gently compress the core shadow just a tiny bit
        float smooth_wrap1 = smoothstep(0.1, 0.9, wrap1);

        // Main directional light
        vec3 main_light = base_color * diffuse * smooth_wrap1;
        vec3 fill_light = base_color * ambient * (wrap2 * 0.5 + 0.5);

        // specular lighting
        // L1
        vec3 R1 = reflect(-L1, N);
        float spec_angle1 = max(dot(V, R1), 0.0);
        float spec_intensity1 = pow(spec_angle1, shininess);

        // L2
        vec3 R2 = reflect(-L2, N);
        float spec_angle2 = max(dot(V, R2), 0.0);
        float spec_intensity2 = pow(spec_angle2, shininess);

        // combine specular
        vec3 specular_light = vec3(1.0) * specular * (spec_intensity1 + spec_intensity2);

        final_color = main_light + fill_light + specular_light;
    }

    if (use_srgb) {
        final_color = pow(clamp(final_color, 0.0, 1.0), vec3(0.45));
    }

    if (use_round_points)
    {
        vec2 uv = 2.0 * gl_PointCoord - 1.0;
        if (dot(uv, uv) > 1.0) discard;
    }

    f_color = vec4(final_color, alpha);
}
)glsl";

static auto* const phong_fshader_normal_dfdx =
#ifndef __EMSCRIPTEN__
    "#version 330 core\n"
#else
    "#version 300 es\n"
#endif
R"glsl(
precision mediump float;

in vec3  v2f_normal;
in vec2  v2f_tex;
in vec3  v2f_view;
in vec3  v2f_color;

uniform bool   use_lighting;
uniform bool   use_texture;
uniform bool   use_srgb;
uniform bool   use_vertex_color;
uniform bool   use_round_points;
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

        vec3 dX = dFdx(v2f_view);
        vec3 dY = dFdy(v2f_view);
        vec3 warped_normal = normalize(cross(dX, dY));
        vec3 N = normalize(gl_FrontFacing ? warped_normal : -warped_normal);
        vec3 L1 = normalize(light1);
        vec3 L2 = normalize(light2);

        // --- Pure Matte "Half-Lambert" Diffuse ---
        // This wraps the light 50% around the dark side, preventing harsh black shadows
        float wrap1 = dot(N, L1) * 0.5 + 0.5;
        float wrap2 = dot(N, L2) * 0.5 + 0.5;

        // Smoothstep to gently compress the core shadow just a tiny bit
        float smooth_wrap1 = smoothstep(0.1, 0.9, wrap1);

        // Main directional light
        vec3 main_light = base_color * diffuse * smooth_wrap1;
        vec3 fill_light = base_color * ambient * (wrap2 * 0.5 + 0.5);
        final_color = main_light + fill_light;
    }

    if (use_srgb) final_color = pow(clamp(final_color, 0.0, 1.0), vec3(0.45));

    if (use_round_points)
    {
        vec2 uv = 2.0 * gl_PointCoord - 1.0;
        if (dot(uv, uv) > 1.0) discard;
    }

    f_color = vec4(final_color, alpha);
}
)glsl";


} // namespace volumetric_base::shaders