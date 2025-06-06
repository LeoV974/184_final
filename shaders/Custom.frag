#version 330

// (Every uniform is available here.)

uniform mat4 u_view_projection;
uniform mat4 u_model;

uniform float u_normal_scaling;
uniform float u_height_scaling;

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

// Feel free to add your own textures. If you need more than 4,
// you will need to modify the skeleton.
uniform sampler2D u_texture_1;
uniform sampler2D u_texture_2;
uniform sampler2D u_texture_3;
uniform sampler2D u_texture_4;

uniform vec4 u_color;

// Environment map! Take a look at GLSL documentation to see how to
// sample from this.
uniform samplerCube u_texture_cubemap;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;


void main() {
  vec3 n = normalize(v_normal.xyz);
  vec3 light_dir = normalize(u_light_pos - v_position.xyz);
  float diffuse = max(dot(n, light_dir), 0.0);
  
  // Toon shading: quantize the diffuse intensity into discrete steps.
  // Here we choose 4 levels
  float levels = 4.0;
  float toonDiffuse = floor(diffuse * levels) / levels;
  // Add a small ambient component so that shadows are not completely dark.
  vec3 ambient = vec3(0.1);
  vec3 lighting = ambient + (u_light_intensity * toonDiffuse);
  vec3 baseColor = u_color.rgb;
  vec3 finalColor = baseColor * lighting;
  out_color = vec4(finalColor, 1.0);
  out_color.a = 1;
}
