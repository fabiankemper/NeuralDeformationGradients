#pragma once

static const char* point_set_vshader =
#ifndef __EMSCRIPTEN__
    "#version 330\n"
#else
    "#version 300 es\n"
#endif
R"glsl(
layout (location=0) in vec4 v_position;
layout (location=1) in vec3 v_normal;
layout (location=2) in vec3 v_color;

out vec3 v2f_normal;
out vec3 v2f_color;
out vec3 v2f_view;

uniform mat4 modelview_projection_matrix;
uniform mat4 modelview_matrix;
uniform mat3 normal_matrix;
uniform float point_size;

void main()
{
    v2f_normal   = normalize(normal_matrix * v_normal);
    v2f_color    = v_color;
    v2f_view     = -(modelview_matrix * v_position).xyz;

    // gl_PointSize = point_size;

    // Scale point size by distance from camera
    vec4 eye_pos = modelview_matrix * v_position;
    gl_PointSize = point_size * ( 1.0 / -eye_pos.z );

    gl_Position  = modelview_projection_matrix * v_position;
}
)glsl";


static const char* point_set_fshader =
#ifndef __EMSCRIPTEN__
    "#version 330\n"
#else
    "#version 300 es\n"
#endif
R"glsl(

in vec3 v2f_normal;
in vec3 v2f_color;
in vec3 v2f_view;

uniform bool   use_lighting;
uniform bool   use_srgb;
uniform float  ambient;
uniform float  diffuse;
uniform float  specular;
uniform float  shininess;
uniform float  alpha;
uniform vec3   light1;
uniform vec3   light2;

//uniform sampler2D mytexture;

out vec4 f_color;

void main()
{
    //vec3 color = use_vertex_color ? v2f_color : (gl_FrontFacing ? front_color : back_color);
    //vec3 color = gl_FrontFacing ? front_color : back_color;
    vec3 color = v2f_color;

    vec3 rgb;

    float d = length(gl_PointCoord-0.5);

    if (d > 0.5)
        discard;

    if (use_lighting)
    {
        vec3 L1 = normalize(light1);
        vec3 L2 = normalize(light2);
        vec3 V  = normalize(v2f_view);
        vec3 N  = gl_FrontFacing ? normalize(v2f_normal) : -normalize(v2f_normal);
        vec3 R;
        float NL, RV;

        rgb = ambient * 0.1 * color;

        NL = dot(N, L1);
        if (NL > 0.0)
        {
            rgb += diffuse * NL * color;
            R  = normalize(-reflect(L1, N));
            RV = dot(R, V);
            if (RV > 0.0)
            {
                rgb += vec3( specular * pow(RV, shininess) );
            }
        }

        NL = dot(N, L2);
        if (NL > 0.0)
        {
            rgb += diffuse * NL * color;
            R  = normalize(-reflect(L2, N));
            RV = dot(R, V);
            if (RV > 0.0)
            {
                rgb += vec3( specular * pow(RV, shininess) );
            }
        }
    }

    // do not use lighting
    else
    {
        rgb = color;
    }

    //if (use_texture) rgb *= texture(mytexture, v2f_tex).xyz;
    if (use_srgb)    rgb  = pow(clamp(rgb, 0.0, 1.0), vec3(0.45));

    f_color = vec4(rgb, alpha);
}
)glsl";
