#version 410

layout(location = 0) in vec4 position;
layout(location = 1) in vec4 color;

void main()
{
	gl_Position = position;
}
