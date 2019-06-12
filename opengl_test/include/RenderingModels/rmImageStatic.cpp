#include "rmImageStatic.h"

static const GLfloat window_positions[] =
{
    // Position i, texcord i
	1.0f,-1.0f,1.0f,0.0f,  // right-down
	-1.0f,-1.0f,0.0f,0.0f, // left-down
	-1.0f,1.0f,0.0f,1.0f,  // left-up
	1.0f,1.0f,1.0f,1.0f    // right-up
};


rmImageStatic::rmImageStatic(std::string _path_Assets_in, std::string image_file_in)
{
    _path_Assets = _path_Assets_in;
    _path_Shaders = _path_Assets + "Shaders/";
    textName = image_file_in;
	Init();
}
rmImageStatic::rmImageStatic(std::string _path_Assets_in, int _ROS_topic_id_in):
    _ROS_topic_id(_ROS_topic_id_in)
{
    _path_Assets = _path_Assets_in;
    _path_Shaders = _path_Assets + "Shaders/";
	Init();
}
void rmImageStatic::Init(){
    //
	_program_ptr.reset( new ShaderProgram() );
    // Load shaders
    _program_ptr->AttachShader(get_full_Shader_path("ImageStatic.vs.glsl"), GL_VERTEX_SHADER);
    _program_ptr->AttachShader(get_full_Shader_path("ImageStatic.fs.glsl"), GL_FRAGMENT_SHADER);
    // Link _program_ptr
	_program_ptr->LinkProgram();
    //

    // Initialize variables
    // Init model matrices
	m_shape.model = glm::mat4(1.0);
    // Colors
    _alpha = 0.7;
    _color_transform = glm::vec4(1.0f);
    //

    // Cache uniform variable id
	uniforms.proj_matrix = glGetUniformLocation(_program_ptr->GetID(), "proj_matrix");
	uniforms.mv_matrix = glGetUniformLocation(_program_ptr->GetID(), "mv_matrix");
    uniforms.color_transform = glGetUniformLocation(_program_ptr->GetID(), "color_transform");
	uniforms.alpha = glGetUniformLocation(_program_ptr->GetID(), "alpha");

    //Load model to shader _program_ptr
	LoadModel();

}
void rmImageStatic::LoadModel(){
    glGenVertexArrays(1, &m_shape.vao);
	glBindVertexArray(m_shape.vao);


	glGenBuffers(1, &m_shape.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(window_positions), window_positions, GL_STATIC_DRAW);


	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(GL_FLOAT) * 4, NULL);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(GL_FLOAT) * 4, (const GLvoid*)(sizeof(GL_FLOAT) * 2) );
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);


    // Texture
	// glEnable(GL_TEXTURE_2D);
	// glActiveTexture(GL_TEXTURE0);
    //Load texture data from file
    std::cout << "start loading <" << textName << ">\n";
	TextureData tdata = Common::Load_png(get_full_Assets_path(textName).c_str());
	glGenTextures(1, &m_shape.m_texture);
	glBindTexture(GL_TEXTURE_2D, m_shape.m_texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, tdata.width, tdata.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, tdata.data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    std::cout << "Load texture success!\n";

}
void rmImageStatic::Update(float dt){
    // Update the data (uniform variables) here
}
void rmImageStatic::Update(ROS_INTERFACE &ros_interface){
    // Update the data (uniform variables) here
}
void rmImageStatic::Render(std::shared_ptr<ViewManager> _camera_ptr){

    glBindVertexArray(m_shape.vao);
	_program_ptr->UseProgram();

    m_shape.model = translateMatrix * rotateMatrix * scaleMatrix;
    // The transformation matrices and projection matrices
    glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, value_ptr( get_mv_matrix(_camera_ptr, m_shape.model) ));
    glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, value_ptr(_camera_ptr->GetProjectionMatrix()));
    //
    glUniform1f(uniforms.alpha, _alpha);
    glUniform4fv(uniforms.color_transform, 1, value_ptr(_color_transform) );

    // glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_shape.m_texture);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);


}
