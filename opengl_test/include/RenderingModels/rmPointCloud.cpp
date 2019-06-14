#include "rmPointCloud.h"


rmPointCloud::rmPointCloud(std::string _path_Assets_in, int _ROS_topic_id_in):
    _ROS_topic_id(_ROS_topic_id_in)
{
    init_paths(_path_Assets_in);
    _max_num_vertex = 5000000; // 5*10^6 // 100000;
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
	m_shape.model = glm::mat4(1.0);
    m_shape.color = glm::vec3(1.0);

    //Load model to shader _program_ptr
	LoadModel();

}
void rmPointCloud::LoadModel(){
    glGenVertexArrays(1, &m_shape.vao);
	glBindVertexArray(m_shape.vao);


	glGenBuffers(1, &m_shape.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo);
	// glBufferData(GL_ARRAY_BUFFER, _max_num_vertex * sizeof(vertex_p_c), NULL, GL_STATIC_DRAW);
	glBufferData(GL_ARRAY_BUFFER, _max_num_vertex * sizeof(vertex_p_c), NULL, GL_DYNAMIC_DRAW); // test, change to dynamic draw to assign point cloud

    // Directly assign data to memory of GPU
    //--------------------------------------------//
	vertex_p_c * vertex_ptr = (vertex_p_c *)glMapBufferRange(GL_ARRAY_BUFFER, 0, _max_num_vertex * sizeof(vertex_p_c), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
	int i;
	for (i = 0; i < _max_num_vertex; i++)
	{
		vertex_ptr[i].position[0] = (random_float() * 2.0f - 1.0f) * 100.0f;
		vertex_ptr[i].position[1] = (random_float() * 2.0f - 1.0f) * 100.0f;
		vertex_ptr[i].position[2] = random_float();
        vertex_ptr[i].color[0] = m_shape.color[0]; //
        vertex_ptr[i].color[1] = m_shape.color[1]; //
        vertex_ptr[i].color[2] = m_shape.color[2]; //
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
    m_shape.indexCount = 0; // 100000; // _max_num_vertex;
    //--------------------------------------------//

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex_p_c), NULL);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertex_p_c), (void *)sizeof(glm::vec3));
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);

    // Texture
	// glEnable(GL_TEXTURE_2D);
	// glActiveTexture(GL_TEXTURE0);
    glGenTextures(1, &m_shape.m_texture);
	glBindTexture(GL_TEXTURE_2D, m_shape.m_texture);
    //Load texture data from file
    std::string _texture_1("star.png");
    std::cout << "start loading <" << _texture_1 << ">\n";
	TextureData tdata = Common::Load_png(get_full_Assets_path(_texture_1).c_str());
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tdata.width, tdata.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, tdata.data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    std::cout << "Load texture success!\n";

}
void rmPointCloud::Update(float dt){
    // Update the data (uniform variables) here
}
void rmPointCloud::Update(ROS_INTERFACE &ros_interface){
    // Update the data (uniform variables) here
    glBindVertexArray(m_shape.vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo); // Start to use the buffer

    // bool pc_result = ros_interface.get_any_pointcloud( _ROS_topic_id, pc_out_ptr);

    // test, use transform
    ros::Time msg_time;
    bool pc_result = ros_interface.get_any_pointcloud( _ROS_topic_id, pc_out_ptr, msg_time);

    // Note: We get the transform update even if there is no new content in for maximum smoothness
    //      (the tf will update even there is no data)
    bool tf_successed = false;
    glm::mat4 _model_tf = ROStf2GLMmatrix(ros_interface.get_tf(_ROS_topic_id, tf_successed, false));
    // glm::mat4 _model_tf = ROStf2GLMmatrix(ros_interface.get_tf(_ROS_topic_id, tf_successed, true, msg_time));
    m_shape.model = _model_tf;
    // Common::print_out_mat4(_model_tf);

    if (pc_result){
        m_shape.indexCount = pc_out_ptr->width;
        if (m_shape.indexCount > _max_num_vertex){
            m_shape.indexCount = _max_num_vertex;
        }
        // vertex_p_c * vertex_ptr = (vertex_p_c *)glMapBufferRange(GL_ARRAY_BUFFER, 0, _max_num_vertex * sizeof(vertex_p_c), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
        vertex_p_c * vertex_ptr = (vertex_p_c *)glMapBufferRange(GL_ARRAY_BUFFER, 0, m_shape.indexCount * sizeof(vertex_p_c), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    	for (size_t i = 0; i < m_shape.indexCount; i++)
    	{
            vertex_ptr[i].position[0] = pc_out_ptr->points[i].x;
    		vertex_ptr[i].position[1] = pc_out_ptr->points[i].y;
    		vertex_ptr[i].position[2] = pc_out_ptr->points[i].z;
    		// If we don't keep udating the color, the color will be lost when resizing the window.
            vertex_ptr[i].color[0] = m_shape.color[0]; //
    		vertex_ptr[i].color[1] = m_shape.color[1]; //
    		vertex_ptr[i].color[2] = m_shape.color[2]; //
    	}
    	glUnmapBuffer(GL_ARRAY_BUFFER);
    }
}
void rmPointCloud::Render(std::shared_ptr<ViewManager> _camera_ptr){

    glBindVertexArray(m_shape.vao);
	_program_ptr->UseProgram();
    // The transformation matrices and projection matrices
    glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, value_ptr( get_mv_matrix(_camera_ptr, m_shape.model) ));
    glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, value_ptr(_camera_ptr->GetProjectionMatrix()));
    // Point sprite
    //--------------------------------//
    glEnable(GL_POINT_SPRITE);
    {
        // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        // glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_shape.m_texture);
        glEnable(GL_PROGRAM_POINT_SIZE);
        glDrawArrays(GL_POINTS, 0, m_shape.indexCount); // draw part of points
    }
    // Close
    glDisable(GL_POINT_SPRITE);
    //--------------------------------//
    // _program_ptr->CloseProgram();
}



void rmPointCloud::set_color(glm::vec3 color_in){
    m_shape.color = color_in;
}
