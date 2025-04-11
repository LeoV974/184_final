#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_1;
uniform vec2 u_texture_1_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
   return texture(u_texture_1, uv).r;
}

void main() {
  // YOUR CODE HERE
  // finding bumped n
  vec3 n = normalize(v_normal.xyz);
  vec3 t = normalize(v_tangent.xyz);
  vec3 b = cross(n,t);
  mat3 TBN = mat3(t,b,n);
  float w = u_texture_1_size[0];
  float h = u_texture_1_size[1];
  // u + 1/w, v
  float dU = (h(v_uv + vec2(1.0/w , 0.0)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
  // u, v + 1/h
  float dV = (h(v_uv + vec2(0.0 , 1.0/h)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
  vec3 n_0 = vec3(-dU, -dV, 1);
  vec3 n_d = normalize(TBN * n_0);
  
  // (Placeholder code. You will want to replace it.)
  vec3 k_a = vec3(0.1);
  vec3 I_a = vec3(1.0);
  vec3 k_d = vec3(1.0);
  vec3 k_s = vec3(0.5);
  float p = 64.0;
  // L = k_a * I_a + k_d * (I/r^2) * max(n.l, 0) + k_s * (I/r^2) * max(n.h, 0)^p


  vec3 light_dir = u_light_pos - v_position.xyz;
  vec3 cam_dir = u_cam_pos - v_position.xyz;
  vec3 l = normalize(light_dir);
  vec3 o = normalize(cam_dir);
  vec3 half_norm = normalize(l + o);
  float r2 = dot(light_dir, light_dir);
  
  vec3 phong = k_a * I_a + k_d * (u_light_intensity / r2) * max(dot(n_d, l), 0.0) + k_s * (u_light_intensity / r2) * pow(max(dot(n_d, half_norm), 0.0), p);
  out_color = vec4(phong, 0.0);
  out_color.a = 1;
}

