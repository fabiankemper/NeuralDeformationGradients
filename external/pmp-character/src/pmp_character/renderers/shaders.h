//=============================================================================
//
//   Exercise code for the lecture
//   "Computer Animation"
//   by Prof. Dr. Mario Botsch, TU Dortmund
//
//   Copyright (C)  Computer Graphics Group, TU Dortmund.
//
//=============================================================================
#pragma once
//=============================================================================

[[maybe_unused]]
static const char *character_vshader =
#ifndef __EMSCRIPTEN__
    "#version 330"
#else
    "#version 300 es"
#endif
    R"glsl(
layout (location=0) in vec4 vertex;
layout (location=1) in vec3 normal;
layout (location=2) in vec2 texcoords;

out vec3 v2f_normal;
out vec2 v2f_texcoords;

uniform mat4 modelview_projection_matrix;
uniform mat3 normal_matrix;

void main()
{
    v2f_normal    = normalize(normal_matrix * normal);
    v2f_texcoords = texcoords;
    gl_Position   = modelview_projection_matrix * vertex;
}
)glsl";

[[maybe_unused]]
static const char *character_fshader =
#ifndef __EMSCRIPTEN__
    "#version 330"
#else
    "#version 300 es"
#endif
    R"glsl(
precision mediump float;

in vec3  v2f_normal;
in vec2  v2f_texcoords;

uniform bool      use_texture;
uniform bool      use_srgb;
uniform sampler2D mytexture;

vec3  light1 = vec3( 1.0, 1.0, 1.0);
vec3  light2 = vec3(-1.0, 1.0, 1.0);
float ambient   = 0.1;
float diffuse   = 1.0;
float specular  = 0.01;
float shininess = 30.0;

out vec4 f_color;

void main()
{
   vec3 color = use_texture ? texture(mytexture, v2f_texcoords).xyz : vec3(0.16, 0.62, 0.87);
   
   vec3 L1 = normalize(light1);
   vec3 L2 = normalize(light2);
   vec3 N  = v2f_normal;
   vec3 V  = vec3(0.0, 0.0, 1.0);
   vec3 R;
   float NL, RV;
   
   vec3 rgb = ambient * 0.1 * color;
   
   NL = dot(N, L1);
   if (NL > 0.0)
   {
       rgb += diffuse * NL * color;
       R  = normalize(-reflect(L1, N));
       RV = dot(R, V);
       if (RV > 0.0) rgb += vec3( specular * pow(RV, shininess) );
   }
   
   NL = dot(N, L2);
   if (NL > 0.0)
   {
        rgb += diffuse * NL * color;
        R  = normalize(-reflect(L2, N));
        RV = dot(R, V);
        if (RV > 0.0) rgb += vec3( specular * pow(RV, shininess) );
   }
   
   if (use_srgb)  rgb = pow(clamp(rgb, 0.0, 1.0), vec3(0.45));
   f_color = vec4(rgb, 1.0);
}
)glsl";

//=============================================================================

static const char *skeleton_vshader =
#ifndef __EMSCRIPTEN__
    "#version 330"
#else
    "#version 300 es"
#endif
    R"glsl(
layout (location=0) in vec4 vertex;
layout (location=1) in vec4 color;

uniform mat4 modelview_projection_matrix;

out vec4 v2f_color;

void main()
{
    gl_PointSize = 10.0;
    v2f_color    = color;
    gl_Position  = modelview_projection_matrix * vertex;
}
)glsl";

static const char *skeleton_fshader =
#ifndef __EMSCRIPTEN__
    "#version 330"
#else
    "#version 300 es"
#endif
    R"glsl(
precision mediump float;
in vec4 v2f_color;
out vec4 out_color;

void main()
{
    out_color = v2f_color;
}
)glsl";

//=============================================================================
