#version 410 core

layout(location = 0) in vec2 position;
layout(location = 1) in vec2 texcoord;

uniform mat4 mv_matrix;
uniform mat4 proj_matrix;
uniform vec4 color_transform;
uniform float alpha;

out VS_OUT
{
    vec4 color_transform;
    float alpha;
	vec2 texcoord;
} vs_out;


void main(void)
{
	// gl_Position = vec4(position, 1.0, 1.0); // Note: z = 1.0 is the farest plane
    // gl_Position = mv_matrix * vec4(position, 0.0, 1.0);
    gl_Position = proj_matrix * mv_matrix * vec4(position, 0.0, 1.0);
	vs_out.texcoord = texcoord;
    vs_out.alpha = alpha;
    vs_out.color_transform = color_transform;
}
