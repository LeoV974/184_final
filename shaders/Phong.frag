#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  vec3 k_a = vec3(0.1);
  vec3 I_a = vec3(1.0);
  vec3 k_d = vec3(1.0);
  vec3 k_s = vec3(0.5);
  float p = 64.0;
  // L = k_a * I_a + k_d * (I/r^2) * max(n.l, 0) + k_s * (I/r^2) * max(n.h, 0)^p

  vec3 n = normalize(v_normal.xyz);
  vec3 light_dir = u_light_pos - v_position.xyz;
  vec3 cam_dir = u_cam_pos - v_position.xyz;
  vec3 l = normalize(light_dir);
  vec3 o = normalize(cam_dir);
  vec3 h = normalize(l + o);
  float r2 = dot(light_dir, light_dir);
  
  vec3 phong = k_a * I_a + k_d * (u_light_intensity / r2) * max(dot(n, l), 0.0) + k_s * (u_light_intensity / r2) * pow(max(dot(n, h), 0.0), p);
  out_color = vec4(phong, 0.0);
  out_color.a = 1;
}

