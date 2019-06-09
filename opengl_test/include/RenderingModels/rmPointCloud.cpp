#include "rmPointCloud.h"


rmPointCloud::rmPointCloud(std::string _path_Assets_in){
    _path_Assets = _path_Assets_in;
    _path_Shaders = _path_Assets + "Shaders/";
    _num_points = 100000;
	Init();
}
void rmPointCloud::Init(){
    //
	_program_ptr.reset( new ShaderProgram() );
    // Load shaders
    _program_ptr->AttachShader(get_full_Shader_path("PointCloud.vs.glsl"), GL_VERTEX_SHADER);
    _program_ptr->AttachShader(get_full_Shader_path("PointCloud.fs.glsl"), GL_FRAGMENT_SHADER);
    // Link _program_ptr
	_program_ptr->LinkProgram();
    //

    // Cache uniform variable id
	uniforms.proj_matrix = glGetUniformLocation(_program_ptr->GetID(), "proj_matrix");
	uniforms.mv_matrix = glGetUniformLocation(_program_ptr->GetID(), "mv_matrix");

    // Init model matrices
	m_shape.model = glm::mat4();

    //Load model to shader _program_ptr
	LoadModel();

}
void rmPointCloud::LoadModel(){
    glGenVertexArrays(1, &m_shape.vao);
	glBindVertexArray(m_shape.vao);


	glGenBuffers(1, &m_shape.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo);
	// glBufferData(GL_ARRAY_BUFFER, _num_points * sizeof(star_t), NULL, GL_STATIC_DRAW);
	glBufferData(GL_ARRAY_BUFFER, _num_points * sizeof(star_t), NULL, GL_DYNAMIC_DRAW); // test, change to dynamic draw to assign point cloud

    // Directly assign data to memory of GPU
    //--------------------------------------------//
	star_t * star = (star_t *)glMapBufferRange(GL_ARRAY_BUFFER, 0, _num_points * sizeof(star_t), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
	int i;
	for (i = 0; i < _num_points; i++)
	{
		star[i].position[0] = (random_float() * 2.0f - 1.0f) * 100.0f;
		star[i].position[1] = (random_float() * 2.0f - 1.0f) * 100.0f;
		star[i].position[2] = random_float();
		star[i].color[0] = 0.8f; //  + random_float() * 0.2f;
		star[i].color[1] = 0.8f; //  + random_float() * 0.2f;
		star[i].color[2] = 0.8f; //  + random_float() * 0.2f;
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
    //--------------------------------------------//

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(star_t), NULL);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(star_t), (void *)sizeof(glm::vec3));
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);


    // Texture
    //Load texture data from file
    std::string _texture_1("star.png");
    std::cout << "start loading <" << _texture_1 << ">\n";
	TextureData tdata = Common::Load_png(get_full_Assets_path(_texture_1).c_str());
	glGenTextures(1, &m_shape.m_texture);
	glBindTexture(GL_TEXTURE_2D, m_shape.m_texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tdata.width, tdata.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, tdata.data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    std::cout << "Load texture success!\n";

}
void rmPointCloud::Update(float dt){

}
void rmPointCloud::Render(std::shared_ptr<ViewManager> _camera_ptr){


}
