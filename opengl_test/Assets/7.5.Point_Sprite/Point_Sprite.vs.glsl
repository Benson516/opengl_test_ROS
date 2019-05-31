#version 410 core

layout(location = 0) in vec4 position;
layout(location = 1) in vec4 color;

uniform float time;
//
uniform mat4 view;
uniform mat4 projection;

flat out vec4 starColor;

void main(void)
{
	// vec4 newVertex = position;

    /*
    // For start only
	newVertex.z += time;
	newVertex.z = fract(newVertex.z);
	float size = (20.0 * newVertex.z * newVertex.z);
    */

    vec4 newVertex = view * position;
    float size = 20.0/(1.0 * abs(newVertex.z) );
	// starColor = smoothstep(1.0, 1.5, size) * color;
    vec4 color_ = color;
    // color_.r = 0.0;
    // color_.g = 0.0;
    starColor = color_;
    starColor.a = 1.0;


    newVertex = projection * newVertex;



    gl_Position = newVertex;
	gl_PointSize = size;
}
