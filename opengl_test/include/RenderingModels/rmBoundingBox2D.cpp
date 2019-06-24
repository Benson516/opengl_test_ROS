#include "rmBoundingBox2D.h"

namespace rmLidarBoundingBox_ns{
    const GLuint box_idx_data[] = {
        0,1,
        1,2,
        2,3,
        3,0
    };
}



rmBoundingBox2D::rmBoundingBox2D(std::string _path_Assets_in, int _ROS_topic_id_in):
    _ROS_topic_id(_ROS_topic_id_in)
{
    init_paths(_path_Assets_in);
    //
    _num_vertex_idx_per_box = 4*2; // 6*(3*2);
    _num_vertex_per_box = 4; // 8;
    _max_num_box = 1000;
    _max_num_vertex_idx = _max_num_box*(long long)(_num_vertex_idx_per_box);
    _max_num_vertex = _max_num_box*(long long)(_num_vertex_per_box);
    //
    generate_box_template();
    //
	Init();
}
void rmBoundingBox2D::Init(){
    //
	_program_ptr.reset( new ShaderProgram() );
    // Load shaders
    _program_ptr->AttachShader(get_full_Shader_path("BoundingBox2D.vs.glsl"), GL_VERTEX_SHADER);
    _program_ptr->AttachShader(get_full_Shader_path("BoundingBox2D.fs.glsl"), GL_FRAGMENT_SHADER);
    // Link _program_ptr
	_program_ptr->LinkProgram();
    //

    // Cache uniform variable id
	uniforms.proj_matrix = glGetUniformLocation(_program_ptr->GetID(), "proj_matrix");
	uniforms.mv_matrix = glGetUniformLocation(_program_ptr->GetID(), "mv_matrix");

    // Init model matrices
	m_shape.model = glm::mat4(1.0);
    attach_pose_model_by_model_ref_ptr(m_shape.model); // For adjusting the model pose by public methods

    //Load model to shader _program_ptr
	LoadModel();

}
void rmBoundingBox2D::LoadModel(){
    glGenVertexArrays(1, &m_shape.vao);
	glBindVertexArray(m_shape.vao);

    glGenBuffers(1, &m_shape.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo);
    glBufferData(GL_ARRAY_BUFFER, _max_num_vertex * sizeof(vertex_p_c_2D), NULL, GL_DYNAMIC_DRAW); // test, change to dynamic draw to assign point cloud
    // Directly assign data to memory of GPU
    //--------------------------------------------//
	vertex_p_c_2D * vertex_ptr = (vertex_p_c_2D *)glMapBufferRange(GL_ARRAY_BUFFER, 0, _max_num_vertex * sizeof(vertex_p_c_2D), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
	size_t _j = 0;
    float box_size = 2.0;
	for (size_t i = 0; i < _max_num_vertex; i++)
	{
		vertex_ptr[i].position[0] = box_size * box_template[_j].position[0] + 1*box_size*(i/_num_vertex_per_box);
		vertex_ptr[i].position[1] = box_size * box_template[_j].position[1] + 1*box_size*(i/_num_vertex_per_box);
		vertex_ptr[i].color[0] = 1.0f; //
		vertex_ptr[i].color[1] = 1.0f; //
		vertex_ptr[i].color[2] = 1.0f; //
        _j++;
        _j = _j%_num_vertex_per_box;
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
    //--------------------------------------------//

    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(vertex_p_c_2D), NULL);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertex_p_c_2D), (void *)sizeof(glm::vec2));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    // ebo
    glGenBuffers(1, &m_shape.ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_shape.ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, _max_num_vertex_idx * sizeof(GLuint), NULL, GL_STATIC_DRAW);
    // Directly assign data to memory of GPU
    //--------------------------------------------//
	GLuint * _idx_ptr = (GLuint *)glMapBufferRange(GL_ELEMENT_ARRAY_BUFFER, 0, _max_num_vertex_idx * sizeof(GLuint), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
	size_t _idx_idx = 0;
    long long _offset_box = 0;
	for (size_t i = 0; i < _max_num_vertex_idx; i++)
	{
        _idx_ptr[i] = rmLidarBoundingBox_ns::box_idx_data[_idx_idx] + _offset_box;
        // std::cout << "_idx_ptr[" << i << "] = " << _idx_ptr[i] << "\n";
        _idx_idx++;
        if (_idx_idx >= _num_vertex_idx_per_box){
            _idx_idx = 0;
            _offset_box += _num_vertex_per_box;
        }
	}
	glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);
    m_shape.indexCount = _max_num_vertex_idx; //  1 * _num_vertex_idx_per_box; // ;
    //--------------------------------------------//




}
void rmBoundingBox2D::Update(float dt){
    // Update the data (buffer variables) here
}
void rmBoundingBox2D::Update(ROS_INTERFACE &ros_interface){
    // Update the data (buffer variables) here

    // test, use transform
    ros::Time msg_time;
    bool _result = ros_interface.get_void_message( _ROS_topic_id, &msg_out_ptr, msg_time);

    if (_result){
        update_GL_data();
    }

    // Move in 3D space
    if (   ros_interface.is_topic_got_frame(_ROS_topic_id)){
        // Note: We get the transform update even if there is no new content in for maximum smoothness
        //      (the tf will update even there is no data)
        bool tf_successed = false;
        glm::mat4 _model_tf = ROStf2GLMmatrix(ros_interface.get_tf(_ROS_topic_id, tf_successed, false));
        // glm::mat4 _model_tf = ROStf2GLMmatrix(ros_interface.get_tf(_ROS_topic_id, tf_successed, true, msg_time));
        // m_shape.model = _model_tf;
        set_pose_modle_ref_by_world(_model_tf);
        // Common::print_out_mat4(_model_tf);
    }

}

void rmBoundingBox2D::Update(ROS_API &ros_api){
    // Update the data (buffer variables) here
    // test, use transform
    ros::Time msg_time;
    bool _result = false;
    // Scops for any_ptr
    {
        boost::any any_ptr;
        _result = ros_api.get_any_message( _ROS_topic_id, any_ptr, msg_time );
        if (_result){
            std::shared_ptr< msgs::CamObj > *_ptr_ptr = boost::any_cast< std::shared_ptr< msgs::CamObj > >( &any_ptr );
            msg_out_ptr = *_ptr_ptr;
        }
    }// end Scops for any_ptr

    if (_result){
        update_GL_data();
    }


    // Move in 3D space
    if (  ros_api.ros_interface.is_topic_got_frame(_ROS_topic_id)){
        ROS_INTERFACE &ros_interface = ros_api.ros_interface;
        // Note: We get the transform update even if there is no new content in for maximum smoothness
        //      (the tf will update even there is no data)
        bool tf_successed = false;
        glm::mat4 _model_tf = ROStf2GLMmatrix(ros_interface.get_tf(_ROS_topic_id, tf_successed, false));
        // glm::mat4 _model_tf = ROStf2GLMmatrix(ros_interface.get_tf(_ROS_topic_id, tf_successed, true, msg_time));
        // m_shape.model = _model_tf;
        set_pose_modle_ref_by_world(_model_tf);
        // Common::print_out_mat4(_model_tf);
    }
}


void rmBoundingBox2D::Render(std::shared_ptr<ViewManager> _camera_ptr){

    glBindVertexArray(m_shape.vao);

	_program_ptr->UseProgram();
    // m_shape.model = translateMatrix * rotateMatrix * scaleMatrix;
    // The transformation matrices and projection matrices
    glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, value_ptr( get_mv_matrix(_camera_ptr, m_shape.model) ));
    glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, value_ptr(_camera_ptr->GetProjectionMatrix()));

    // Setting
    // glLineWidth(5.0);

    // Draw the element according to ebo
    // glDrawElements(GL_TRIANGLES, m_shape.indexCount, GL_UNSIGNED_INT, 0);
    glDrawElements(GL_LINES, m_shape.indexCount, GL_UNSIGNED_INT, 0);
    // glDrawArrays(GL_TRIANGLES, 0, 3*5); // draw part of points
    //--------------------------------//
    _program_ptr->CloseProgram();
}


void rmBoundingBox2D::update_GL_data(){

    if (msg_out_ptr->camObj.size() == 0){
        m_shape.indexCount = 0;
        return;
    }

    long long num_box = msg_out_ptr->camObj.size();
    if (num_box > _max_num_box){
        num_box = _max_num_box;
    }

    // vao vbo
    glBindVertexArray(m_shape.vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo); // Start to use the buffer


    // vertex_p_c_2D * vertex_ptr = (vertex_p_c_2D *)glMapBufferRange(GL_ARRAY_BUFFER, 0, _max_num_vertex * sizeof(vertex_p_c_2D), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    vertex_p_c_2D * vertex_ptr = (vertex_p_c_2D *)glMapBufferRange(GL_ARRAY_BUFFER, 0, num_box * _num_vertex_per_box * sizeof(vertex_p_c_2D), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    float box_size = 2.0;
    size_t _j = 0;
    size_t _box_count = 0;
	for (size_t i = 0; i < num_box; i++)
	{
        //
        auto & _box = msg_out_ptr->camObj[i];
        box_param_cv _a_box_param_cv(_box.x, _box.y, _box.width, _box.height, _box.cls);
        box_param_gl _a_box_param_gl;
        convert_cv_to_normalized_gl(_a_box_param_cv, _a_box_param_gl);
        if (!is_gl_box_valid(_a_box_param_gl)){
            continue; // Don't add to buffer
        }
        _box_count++;
        //
        for (size_t _k=0; _k <_num_vertex_per_box; ++_k ){
            vertex_ptr[_j].position[0] = _a_box_param_gl.xy_list[_k][0];
    		vertex_ptr[_j].position[1] = _a_box_param_gl.xy_list[_k][1];
    		vertex_ptr[_j].color[0] = 0.0f; //
    		vertex_ptr[_j].color[1] = 1.0f; //
    		vertex_ptr[_j].color[2] = 0.0f; //
            _j++;
        }
        //
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
    //
    num_box = _box_count;
    m_shape.indexCount = num_box*_num_vertex_idx_per_box;
    //--------------------------------------------//
}
