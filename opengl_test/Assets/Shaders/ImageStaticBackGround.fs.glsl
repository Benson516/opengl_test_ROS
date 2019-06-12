#version 410 core

uniform sampler2D tex;

out vec4 color;


in VS_OUT{
    vec4 color_transform;
    float alpha;
	vec2 texcoord;
} fs_in;

void main(void)
{
	vec4 texture_color = texture(tex, fs_in.texcoord);
	//
    texture_color = fs_in.color_transform * texture_color;
    if (fs_in.alpha >= 0.0){
        texture_color.a = fs_in.alpha;
    }
    color = texture_color;

}
