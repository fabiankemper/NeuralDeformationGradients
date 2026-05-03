#pragma once
#include <string>

static const std::string matcap_vshader = R"(
#version 330 core

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 normal;

uniform mat4 modelview_matrix;
uniform mat4 modelview_projection_matrix;
uniform mat4 normal_matrix;

out vec3 v_normal;
out vec3 v_view_normal;

void main()
{
    gl_Position = modelview_projection_matrix * vec4(position, 1.0);
    v_normal = normalize(normal_matrix * normal);

    // transform normal into view space
    v_view_normal = normalize((modelview_matrix * vec4(normal, 0.0)).xyz);
}
)";

static const std::string matcap_fshader = R"(
#version 330 core

in vec3 v_view_normal;
out vec4 frag_color;

uniform sampler2D matcap_texture;
uniform float alpha;

void main()
{
    // normalize view-space normal
    vec3 n = normalize(v_view_normal);

    // compute reflection vector assuming camera at origin
    vec3 r = n;

    // convert reflection vector to texture coordinates ([-1, 1] -> [0, 1])
    float u = r.x * 0.5 + 0.5;
    float v = r.y * 0.5 + 0.5;

    // sample matcap texture
    frag_color = texture(matcap_texture, vec2(u, v));
    frag_color.a *= alpha;
}
)";
