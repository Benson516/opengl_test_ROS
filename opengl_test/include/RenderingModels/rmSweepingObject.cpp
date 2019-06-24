#include "rmSweepingObject.h"
#include <math.h>       /* cos */



rmSweepingObject::rmSweepingObject(std::string _path_Assets_in){
    _path_Shaders_sub_dir += "SweepObject/";
    init_paths(_path_Assets_in);
    //
    _max_num_vertex_of_curve = 100;
    //
	Init();
}
rmSweepingObject::rmSweepingObject(std::string _path_Assets_in, int _ROS_topic_id_in):
    _ROS_topic_id(_ROS_topic_id_in)
{
    _path_Shaders_sub_dir += "SweepObject/";
    init_paths(_path_Assets_in);
    //
    _max_num_vertex_of_curve = 100;
    //
	Init();
}
void rmSweepingObject::Init(){
    //
	_program_ptr.reset( new ShaderProgram() );
    // Load shaders
    _program_ptr->AttachShader(get_full_Shader_path("SweepObject.vs.glsl"), GL_VERTEX_SHADER);
    _program_ptr->AttachShader(get_full_Shader_path("SweepObject.gs.glsl"), GL_GEOMETRY_SHADER);
    _program_ptr->AttachShader(get_full_Shader_path("SweepObject.fs.glsl"), GL_FRAGMENT_SHADER);
    // Link _program_ptr
	_program_ptr->LinkProgram();
    //

    // Cache uniform variable id
	uniforms.proj_matrix = glGetUniformLocation(_program_ptr->GetID(), "proj_matrix");
	uniforms.mv_matrix = glGetUniformLocation(_program_ptr->GetID(), "mv_matrix");
    //
    uniforms.lookat_matrix.resize(_max_num_vertex_of_curve);
    for (int i = 0; i < _max_num_vertex_of_curve; i++) {
		uniforms.lookat_matrix[i] = glGetUniformLocation(_program_ptr->GetID(), std::string("lookat_matrix[" + std::to_string(i) + "]").c_str());
	}

    // Init model matrices
	m_shape.model = glm::mat4(1.0);
    attach_pose_model_by_model_ref_ptr(m_shape.model); // For adjusting the model pose by public methods


    //Load model to shader _program_ptr
	LoadModel();

}
void rmSweepingObject::LoadModel(){

    _curve_Points.push_back( glm::vec3(0.00, 0.00, 0.00) );
    _curve_Points.push_back( glm::vec3(0.05, 0.00, 0.63) );
    _curve_Points.push_back( glm::vec3(0.20, 0.00, 1.24) );
    _curve_Points.push_back( glm::vec3(0.44, 0.00, 1.82) );
    _curve_Points.push_back( glm::vec3(0.76, 0.00, 2.35) );
    _curve_Points.push_back( glm::vec3(1.17, 0.00, 2.83) );
    _curve_Points.push_back( glm::vec3(1.65, 0.00, 3.24) );
    _curve_Points.push_back( glm::vec3(2.18, 0.00, 3.56) );
    _curve_Points.push_back( glm::vec3(2.76, 0.00, 3.80) );
    _curve_Points.push_back( glm::vec3(3.37, 0.00, 3.95) );
    _curve_Points.push_back( glm::vec3(4.00, 0.00, 4.00) );


    // Calculate rotaion matrices and scales
    for (int i = 0; i < _curve_Points.size(); i++) {
		glm::mat4 temp(1.0);
		temp = rotate(temp, deg2rad(i * 9.0f), glm::vec3(0, 1, 0));
		temp = scale(temp, glm::vec3(1, 1, 1));
		lookat_matrix[i] = temp;
	}
    //

    // m_shape.indexCount = _max_num_vertex_of_curve; //  1 * _num_vertex_idx_per_box; // ;
    m_shape.indexCount = _curve_Points.size();

    glGenVertexArrays(1, &m_shape.vao);
	glBindVertexArray(m_shape.vao);

    glGenBuffers(1, &m_shape.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo);
    glBufferData(GL_ARRAY_BUFFER, _max_num_vertex_of_curve * sizeof(vertex_p_c), NULL, GL_DYNAMIC_DRAW); // test, change to dynamic draw to assign point cloud
    // Directly assign data to memory of GPU
    //--------------------------------------------//
	vertex_p_c * vertex_ptr = (vertex_p_c *)glMapBufferRange(GL_ARRAY_BUFFER, 0, _max_num_vertex_of_curve * sizeof(vertex_p_c), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    double _size = 1.0;
    for (size_t i = 0; i < m_shape.indexCount; i++)
	{
		vertex_ptr[i].position[0] = _size*_curve_Points[i][0];
		vertex_ptr[i].position[1] = _size*_curve_Points[i][1];
		vertex_ptr[i].position[2] = _size*_curve_Points[i][2];
		vertex_ptr[i].color[0] = 1.0f; //
		vertex_ptr[i].color[1] = 1.0f; //
		vertex_ptr[i].color[2] = 1.0f; //
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
    //--------------------------------------------//

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex_p_c), NULL);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertex_p_c), (void *)sizeof(glm::vec3));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);




}
void rmSweepingObject::Update(float dt){
    // Update the data (buffer variables) here
}
void rmSweepingObject::Update(ROS_INTERFACE &ros_interface){
    // Update the data (buffer variables) here
}

void rmSweepingObject::Update(ROS_API &ros_api){
    // Update the data (buffer variables) here
}


void rmSweepingObject::Render(std::shared_ptr<ViewManager> _camera_ptr){

    glBindVertexArray(m_shape.vao);

	_program_ptr->UseProgram();
    // The transformation matrices and projection matrices
    glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, value_ptr( get_mv_matrix(_camera_ptr, m_shape.model) ));
    glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, value_ptr(_camera_ptr->GetProjectionMatrix()));
    //
    for (int i = 0; i < m_shape.indexCount; i++) {
		glUniformMatrix4fv( uniforms.lookat_matrix[i], 1, GL_FALSE, value_ptr(lookat_matrix[i]) );
	}
    //
    // Draw the element according to ebo
    // glDrawElements(GL_LINE_STRIP, m_shape.indexCount, GL_UNSIGNED_INT, 0);
    glDrawArrays(GL_LINE_STRIP, 0, m_shape.indexCount); // draw part of points
    //--------------------------------//
    _program_ptr->CloseProgram();
}


void rmSweepingObject::update_GL_data(){

}
