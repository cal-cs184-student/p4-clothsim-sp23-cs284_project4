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
  float ka = 0.1;
  float kd = 1.0;
  float ks = 0.8;
  float Ia = 0.1;
  float p = 50.0;
  
  // Phong shading
  vec3 displacementToLight = u_light_pos - v_position.xyz;
  vec3 Disp2Cam = u_cam_pos - v_position.xyz;
  vec3 i = normalize(displacementToLight);
  vec3 outgoing = normalize(Disp2Cam);
  vec3 halfvec =  normalize(i + outgoing);
  
  float r = length(displacementToLight), r2 = r*r;

  float wd = max(dot(v_normal.xyz, i), 0.0) / r2, ws = pow(max(dot(v_normal.xyz, halfvec), 0.0), p) / r2;
  vec3 color = ka * Ia + (kd * wd + ks * ws) * u_light_intensity;
  

  out_color = vec4(color, 0.0);
  out_color.a = 1.0;
}

