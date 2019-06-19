#version 410

layout(lines, invocations = 1) in;
layout(triangle_strip, max_vertices = 22) out;

in int gl_PrimitiveIDIn[];
out vec4 gsOutColor;

uniform mat4 mv_matrix;
uniform mat4 proj_matrix;
// The list of rotation matrices for direction
uniform mat4 lookat_matrix[11];

//
const float PI = 3.1415926;

void main()
{
	mat4 mvp_matrix =  proj_matrix * mv_matrix;

	vec4 up = vec4(0, 1, 0, 0);
	vec4 right = vec4(1, 0, 0, 0);
	vec4 nowRight = lookat_matrix[gl_PrimitiveIDIn[0]] * right;
	vec4 nextRight = lookat_matrix[gl_PrimitiveIDIn[0] + 1] *  right;

	float count = 10;
	float angleOffset = (360.0 / count) * (PI / 180.0);
	for (int i = 0; i <= count; i++) {
		float nowAngle = i * angleOffset;

		float x = cos(nowAngle);
		float y = sin(nowAngle);

		gl_Position = mvp_matrix * (gl_in[0].gl_Position + nowRight * x + up * y);
		gsOutColor = vec4(i / count, 0, 0, 1.0);
		EmitVertex();
		gl_Position = mvp_matrix * (gl_in[1].gl_Position + nextRight * x + up * y);
		gsOutColor = vec4(i / count, 0, 0, 1.0);
		EmitVertex();
	}
	EndPrimitive();
}
