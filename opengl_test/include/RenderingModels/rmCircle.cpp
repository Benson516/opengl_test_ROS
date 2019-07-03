#include "rmCircle.h"





rmCircle::rmCircle(std::string _path_Assets_in, int _ROS_topic_id_in):
    _ROS_topic_id(_ROS_topic_id_in)
{
    _path_Shaders_sub_dir += "Circle/";
    init_paths(_path_Assets_in);
    //
    _num_vertex_per_box = 1;
    _max_num_box = 1000;
    _max_num_vertex = _max_num_box*(long long)(_num_vertex_per_box);
    //
	Init();
}
void rmCircle::Init(){
    //
	_program_ptr.reset( new ShaderProgram() );
    // Load shaders
    _program_ptr->AttachShader(get_full_Shader_path("Circle.vs.glsl"), GL_VERTEX_SHADER);
    _program_ptr->AttachShader(get_full_Shader_path("Circle.gs.glsl"), GL_VERTEX_SHADER);
    _program_ptr->AttachShader(get_full_Shader_path("Circle.fs.glsl"), GL_FRAGMENT_SHADER);
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
void rmCircle::LoadModel(){
    glGenVertexArrays(1, &m_shape.vao);
	glBindVertexArray(m_shape.vao);

    glGenBuffers(1, &m_shape.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo);
    glBufferData(GL_ARRAY_BUFFER, _max_num_vertex * sizeof(vertex_p_c), NULL, GL_DYNAMIC_DRAW); // test, change to dynamic draw to assign point cloud
    // Directly assign data to memory of GPU
    //--------------------------------------------//
	vertex_p_c * vertex_ptr = (vertex_p_c *)glMapBufferRange(GL_ARRAY_BUFFER, 0, _max_num_vertex * sizeof(vertex_p_c), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
	size_t _j = 0;
    float radious = 1.0;
	for (size_t i = 0; i < _max_num_box; i++)
	{
        // Center
		vertex_ptr[_j].position[0] = 2.0*radious*(i/_num_vertex_per_box);
		vertex_ptr[_j].position[1] = 0.0f;
		vertex_ptr[_j].position[2] = 0.0f;
        vertex_ptr[_j].radious     = radious;
		vertex_ptr[_j].color[0] = 1.0f; //
		vertex_ptr[_j].color[1] = 1.0f; //
		vertex_ptr[_j].color[2] = 1.0f; //
        _j++;
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
    //--------------------------------------------//

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex_p_c), NULL);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(vertex_p_c), (void *)(sizeof(glm::vec3) )  );
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(vertex_p_c), (void *)(sizeof(glm::vec3) + sizeof(float) )  );
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);

    m_shape.indexCount = _max_num_vertex;
    //--------------------------------------------//




}
void rmCircle::Update(float dt){
    // Update the data (buffer variables) here
}
void rmCircle::Update(ROS_INTERFACE &ros_interface){
    // Update the data (buffer variables) here
}

void rmCircle::Update(ROS_API &ros_api){
    // Update the data (buffer variables) here

}


void rmCircle::Render(std::shared_ptr<ViewManager> _camera_ptr){

    glBindVertexArray(m_shape.vao);

	_program_ptr->UseProgram();
    // The transformation matrices and projection matrices
    glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, value_ptr( get_mv_matrix(_camera_ptr, m_shape.model) ));
    glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, value_ptr(_camera_ptr->GetProjectionMatrix()));
    // Draw the element according to ebo
    // glDrawElements(GL_TRIANGLES, m_shape.indexCount, GL_UNSIGNED_INT, 0);
    glDrawArrays(GL_POINTS, 0, m_shape.indexCount); // draw part of points
    //--------------------------------//
    _program_ptr->CloseProgram();
}


void rmCircle::update_GL_data(){
    long long num_box = msg_out_ptr->lidRoiBox.size();
    if (num_box > _max_num_box){
        num_box = _max_num_box;
    }

    // vao vbo
    glBindVertexArray(m_shape.vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo); // Start to use the buffer


    m_shape.indexCount = num_box;
    vertex_p_c * vertex_ptr = (vertex_p_c *)glMapBufferRange(GL_ARRAY_BUFFER, 0, _max_num_vertex * sizeof(vertex_p_c), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    // vertex_p_c * vertex_ptr = (vertex_p_c *)glMapBufferRange(GL_ARRAY_BUFFER, 0, num_box * _num_vertex_per_box * sizeof(vertex_p_c), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    auto * _point_ptr = &(msg_out_ptr->lidRoiBox[0].p0);
    size_t _j = 0;
    for (size_t i = 0; i < num_box; i++)
    {
            //
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);
}
