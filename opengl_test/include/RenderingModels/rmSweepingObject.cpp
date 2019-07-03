#include "rmSweepingObject.h"
#include <math.h>       /* cos */



rmSweepingObject::rmSweepingObject(std::string _path_Assets_in){
    _path_Shaders_sub_dir += "SweepObject/";
    init_paths(_path_Assets_in);
    //
    _max_num_vertex_of_curve = 100;
    _max_num_vertex_of_shape = 50;
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
    _max_num_vertex_of_shape = 50;
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
    //
    uniforms.section_vertexes.resize(_max_num_vertex_of_shape);
    for (int i = 0; i < _max_num_vertex_of_shape; i++) {
		uniforms.section_vertexes[i] = glGetUniformLocation(_program_ptr->GetID(), std::string("section_vertexes[" + std::to_string(i) + "]").c_str());
	}
    uniforms._num_vertex_of_shape = glGetUniformLocation(_program_ptr->GetID(), "_num_vertex_of_shape");
    //

    // Init model matrices
	m_shape.model = glm::mat4(1.0);
    attach_pose_model_by_model_ref_ptr(m_shape.model); // For adjusting the model pose by public methods


    //Load model to shader _program_ptr
	LoadModel();

}
void rmSweepingObject::LoadModel(){

    section_vertexes.resize(4);
    /*
    section_vertexes[0] = glm::vec3(-1.0f, -1.0f, 0.0f);
    section_vertexes[1] = glm::vec3(-1.0f, 1.0f, 0.0f);
    section_vertexes[2] = glm::vec3(1.0f, 1.0f, 0.0f);
    section_vertexes[3] = glm::vec3(1.0f, -1.0f, 0.0f);
    */
    /*
    section_vertexes[0] = glm::vec3(-1.0f, 0.0f, -1.0f);
    section_vertexes[1] = glm::vec3(-1.0f, 0.0f, 1.0f);
    section_vertexes[2] = glm::vec3(1.0f, 0.0f, 1.0f);
    section_vertexes[3] = glm::vec3(1.0f, 0.0f, -1.0f);
    */
    section_vertexes[0] = glm::vec3(0.0f, -1.0f, -1.0f);
    section_vertexes[1] = glm::vec3(0.0f, -1.0f, 1.0f);
    section_vertexes[2] = glm::vec3(0.0f, 1.0f, 1.0f);
    section_vertexes[3] = glm::vec3(0.0f, 1.0f, -1.0f);

    glm::mat4 _delta_T = glm::scale(glm::mat4(1.0), glm::vec3(1.0f, 0.01f, 1.0f) );
    for (size_t i=0; i < section_vertexes.size(); ++i){
        section_vertexes[i] = (_delta_T * glm::vec4(section_vertexes[i], 1.0f)).xyz();
    }

    //
    _curve_Points.push_back( glm::vec3(0.00, 0.00, 0.00) );
    /*
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
    _curve_Points.push_back( glm::vec3(5.00, 0.00, 5.00) );
    _curve_Points.push_back( glm::vec3(6.00, 0.00, 6.00) );
    _curve_Points.push_back( glm::vec3(7.00, 0.00, 7.00) );
    _curve_Points.push_back( glm::vec3(8.00, 0.00, 8.00) );
    */

    for (int i=1; i < 20; ++i){
        _curve_Points.push_back( _curve_Points[i-1] + glm::vec3(1.0f, 0.0f + float(i/10.0f), 0.0f) );
    }

    // Initial heading
    // glm::vec3 _d_initial(0.0f, 0.0f, 1.0f);

    // Calculate rotaion matrices and scales
    // Calculate elements of lookat_matrix_list (new method, automatically generated according to curve)
    lookat_matrix_list.resize(_max_num_vertex_of_curve);
    lookat_matrix_list[0] = glm::mat4(1.0f);


    if ( _curve_Points.size() > 1){
        glm::mat4 _accumulated_T(1.0f);
        glm::vec3 _d_pre = glm::normalize(_curve_Points[1] - _curve_Points[0]);
        for (size_t i=1; i < (_curve_Points.size() - 1); ++i){
            // direction
            glm::vec3 _d_next = glm::normalize( _curve_Points[i+1] - _curve_Points[i] );
            // angle
            GLfloat _angle = glm::acos( glm::dot(_d_pre, _d_next) ); // Note: ||_d_pre|| = ||_d_next|| = 1
            //
            if ( _angle < 0.000017 ){ // 0.001 deg
                lookat_matrix_list[i] = lookat_matrix_list[i-1];
            }else{
                // axis - cross
                glm::vec3 _axis = glm::cross(_d_pre, _d_next);
                glm::vec3 _axis_n = glm::normalize(_axis);
                //
                glm::mat4 _delta_rot = glm::rotate(glm::mat4(1.0f), _angle/2.0f, _axis_n);
                lookat_matrix_list[i] = _delta_rot * _accumulated_T;
                _accumulated_T = _delta_rot * lookat_matrix_list[i];
                //
            }
            _d_pre = _d_next;
        }
        lookat_matrix_list[_curve_Points.size() - 1] = _accumulated_T;
    }


    /*
    if ( _curve_Points.size() > 1){
        glm::vec3 _d_pre = glm::normalize( _d_initial );
        for (size_t i=1; i < (_curve_Points.size() ); ++i){
            // direction
            glm::vec3 _d_next = glm::normalize( _curve_Points[i] - _curve_Points[i-1] );
            // angle
            GLfloat _angle = glm::acos( glm::dot(_d_pre, _d_next) ); // Note: ||_d_pre|| = ||_d_next|| = 1
            if (_angle < 0.000017){ // 0.001 deg
                lookat_matrix_list[i] = lookat_matrix_list[i-1];
            }else{
                // axis - cross
                glm::vec3 _axis = glm::cross(_d_pre, _d_next);
                glm::vec3 _axis_n = glm::normalize(_axis);
                //
                glm::mat4 _delta_rot = glm::rotate(glm::mat4(1.0f), _angle, _axis_n);
                lookat_matrix_list[i] = _delta_rot * lookat_matrix_list[i-1];
            }
            //
            _d_pre = _d_next;
        }
    }
    */



    /*
    // Calculate elements of lookat_matrix_list (old method, hard coded)
    for (int i = 0; i < _curve_Points.size(); ++i) {
		glm::mat4 temp(1.0);
		temp = rotate(temp, deg2rad(i * 9.0f), glm::vec3(0, 1, 0));
		temp = scale(temp, glm::vec3(1, 1, 1));
		lookat_matrix_list[i] = temp;
	}
    //
    */

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
		vertex_ptr[i].color[0] = 1.0f * (float(m_shape.indexCount - i))/ float(m_shape.indexCount); //
		vertex_ptr[i].color[1] = 0.5f; //
		vertex_ptr[i].color[2] = 1.0f * (float(i))/ float(m_shape.indexCount); //
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


void rmSweepingObject::Render(std::shared_ptr<ViewManager> &_camera_ptr){

    glBindVertexArray(m_shape.vao);

	_program_ptr->UseProgram();
    // The transformation matrices and projection matrices
    glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, value_ptr( get_mv_matrix(_camera_ptr, m_shape.model) ));
    glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, value_ptr(_camera_ptr->GetProjectionMatrix()));
    //
    for (int i = 0; i < m_shape.indexCount; i++) {
		glUniformMatrix4fv( uniforms.lookat_matrix[i], 1, GL_FALSE, value_ptr(lookat_matrix_list[i]) );
	}
    for (int i = 0; i < section_vertexes.size(); i++) {
		glUniform3fv( uniforms.section_vertexes[i], 1, value_ptr(section_vertexes[i]) );
	}
    glUniform1i(uniforms._num_vertex_of_shape, int(section_vertexes.size()) );
    //
    // Draw the elements
    glDrawArrays(GL_LINE_STRIP, 0, m_shape.indexCount); // draw part of points
    //--------------------------------//
    _program_ptr->CloseProgram();
}


void rmSweepingObject::update_GL_data(){

}
