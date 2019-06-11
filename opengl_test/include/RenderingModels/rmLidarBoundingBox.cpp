#include "rmLidarBoundingBox.h"


rmLidarBoundingBox::rmLidarBoundingBox(std::string _path_Assets_in, int _ROS_topic_id_in):
    _ROS_topic_id(_ROS_topic_id_in)
{
    _path_Assets = _path_Assets_in;
    _path_Shaders = _path_Assets + "Shaders/";
    //
    _num_vertex_idx_per_box = 6*(3*2);
    _num_vertex_per_box = 8;
    _max_num_box = 1000;
    _max_num_vertex_idx = _max_num_box*(long long)(_num_vertex_idx_per_box);
    _max_num_vertex = _max_num_box*(long long)(_num_vertex_per_box);
    //
    generate_box_template();
    //
	Init();
}
void rmLidarBoundingBox::Init(){
    //
	_program_ptr.reset( new ShaderProgram() );
    // Load shaders
    _program_ptr->AttachShader(get_full_Shader_path("LidarBoundingBox.vs.vs.glsl"), GL_VERTEX_SHADER);
    _program_ptr->AttachShader(get_full_Shader_path("LidarBoundingBox.vs.fs.glsl"), GL_FRAGMENT_SHADER);
    // Link _program_ptr
	_program_ptr->LinkProgram();
    //

    // Cache uniform variable id
	uniforms.proj_matrix = glGetUniformLocation(_program_ptr->GetID(), "proj_matrix");
	uniforms.mv_matrix = glGetUniformLocation(_program_ptr->GetID(), "mv_matrix");

    // Init model matrices
	m_shape.model = glm::mat4(1.0);

    //Load model to shader _program_ptr
	LoadModel();

}
void rmLidarBoundingBox::LoadModel(){
    glGenVertexArrays(1, &m_shape.vao);
	glBindVertexArray(m_shape.vao);

    glGenBuffers(1, &m_shape.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo);
    glBufferData(GL_ARRAY_BUFFER, _max_num_vertex * sizeof(vertex_p_c), NULL, GL_DYNAMIC_DRAW); // test, change to dynamic draw to assign point cloud
    // Directly assign data to memory of GPU
    //--------------------------------------------//
	vertex_p_c * vertex_ptr = (vertex_p_c *)glMapBufferRange(GL_ARRAY_BUFFER, 0, _max_num_vertex * sizeof(vertex_p_c), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
	size_t _j = 0;
	for (size_t i = 0; i < _max_num_vertex; i++)
	{
		vertex_ptr[i].position[0] = box_template[_j].position[0];
		vertex_ptr[i].position[1] = box_template[_j].position[1];
		vertex_ptr[i].position[2] = box_template[_j].position[2];
		vertex_ptr[i].color[0] = 1.0f; //
		vertex_ptr[i].color[1] = 1.0f; //
		vertex_ptr[i].color[2] = 1.0f; //
        _j++;
        _j = _j%_num_vertex_per_box;
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
    m_shape.indexCount = _max_num_vertex;
    //--------------------------------------------//

    // ebo
    glGenBuffers(1, &m_shape.ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_shape.vbo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, _max_num_vertex * sizeof(rmLidarBoundingBox_ns::box_idx_data), NULL, GL_STATIC_DRAW);
    // Directly assign data to memory of GPU
    //--------------------------------------------//
	GLshort * _idx_ptr = (GLshort *)glMapBufferRange(GL_ARRAY_BUFFER, 0, _max_num_vertex * sizeof(rmLidarBoundingBox_ns::box_idx_data), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
	size_t _idx_idx = 0;
    long long _offset_box = 0;
	for (size_t i = 0; i < _max_num_vertex_idx; i++)
	{
        _idx_ptr[i] = rmLidarBoundingBox_ns::box_idx_data[_idx_idx] + _offset_box;
        _idx_idx++;
        if (_idx_idx >= _num_vertex_idx_per_box){
            _idx_idx = 0;
            _offset_box += _num_vertex_idx_per_box;
        }
	}
	glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);
    //--------------------------------------------//

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex_p_c), NULL);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertex_p_c), (void *)sizeof(glm::vec3));
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

}
void rmLidarBoundingBox::Update(float dt){
    // Update the data (uniform variables) here
}
void rmLidarBoundingBox::Update(ROS_INTERFACE &ros_interface){

}
void rmLidarBoundingBox::Render(std::shared_ptr<ViewManager> _camera_ptr){


}
