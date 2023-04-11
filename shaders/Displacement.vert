#version 330

uniform mat4 u_view_projection;
uniform mat4 u_model;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 in_position;
in vec4 in_normal;
in vec4 in_tangent;
in vec2 in_uv;

out vec4 v_position;
out vec4 v_normal;
out vec2 v_uv;
out vec4 v_tangent;

float h(vec2 uv) {
  // You may want to use this helper function...
    vec4 temp = texture(u_texture_2, uv);
  return temp.r;
}

void main() {
  // YOUR CODE HERE

  v_position = u_model * in_position;
  v_normal = normalize(u_model * in_normal);
  v_uv = in_uv;
  v_tangent = normalize(u_model * in_tangent);

  float k_h = u_height_scaling, H_uv = h(v_uv);
  vec3 pos = vec3(v_position), n = vec3(v_normal);
  pos = pos + n * H_uv * k_h / 3.0;
  v_position = vec4(pos, v_position.w);
  gl_Position = u_view_projection * v_position;
}