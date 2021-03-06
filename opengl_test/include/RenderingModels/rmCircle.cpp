#include "rmCircle.h"





rmCircle::rmCircle(std::string _path_Assets_in, std::string frame_id_in):
    _frame_id(frame_id_in)
{
    _path_Shaders_sub_dir += "Circle/";
    init_paths(_path_Assets_in);
    //
    _num_vertex_per_shape = 1;
    _max_num_shape = 1000;
    _max_num_vertex = _max_num_shape*(long long)(_num_vertex_per_shape);
    //
	Init();
}
rmCircle::rmCircle(std::string _path_Assets_in, int _ROS_topic_id_in):
    _ROS_topic_id(_ROS_topic_id_in)
{
    _path_Shaders_sub_dir += "Circle/";
    init_paths(_path_Assets_in);
    //
    _num_vertex_per_shape = 1;
    _max_num_shape = 1000;
    _max_num_vertex = _max_num_shape*(long long)(_num_vertex_per_shape);
    //
	Init();
}
void rmCircle::Init(){
    //
	_program_ptr.reset( new ShaderProgram() );
    // Load shaders
    _program_ptr->AttachShader(get_full_Shader_path("Circle.vs.glsl"), GL_VERTEX_SHADER);
    _program_ptr->AttachShader(get_full_Shader_path("Circle.gs.glsl"), GL_GEOMETRY_SHADER);
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
    _line_width = 1.0f;

    //Load model to shader _program_ptr
	LoadModel();

}
void rmCircle::LoadModel(){
    glGenVertexArrays(1, &m_shape.vao);
	glBindVertexArray(m_shape.vao);

    glGenBuffers(1, &m_shape.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo);
    glBufferData(GL_ARRAY_BUFFER, _max_num_vertex * sizeof(circle_data), NULL, GL_DYNAMIC_DRAW); // test, change to dynamic draw to assign point cloud
    // Directly assign data to memory of GPU
    //--------------------------------------------//
	circle_data * vertex_ptr = (circle_data *)glMapBufferRange(GL_ARRAY_BUFFER, 0, _max_num_vertex * sizeof(circle_data), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
	size_t _j = 0;
    float radious = 1.0;
	for (size_t i = 0; i < _max_num_shape; i++)
	{
        // Center
		vertex_ptr[_j].position[0] = 2.0*radious*(i/_num_vertex_per_shape);
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

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(circle_data), NULL);
    glVertexAttribPointer(1, 1, GL_FLOAT, GL_FALSE, sizeof(circle_data), (void *)(sizeof(glm::vec3) )  );
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(circle_data), (void *)(sizeof(glm::vec3) + sizeof(float) )  );
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);
    glEnableVertexAttribArray(2);

    // m_shape.indexCount = _max_num_vertex;
    m_shape.indexCount = 0;
    //--------------------------------------------//


    /*
    // test
    vector<circle_data> data_test;
    float __radious = 0.5;
    for (size_t _k=0; _k < 10; ++_k){
        data_test.push_back(
            circle_data(
                glm::vec3(10.0f, -2.0*__radious*(_k/_num_vertex_per_shape), 0.0f),
                __radious,
                glm::vec3(0.6f, 0.6f, 0.2f)
            )
        );
        insert_circle(data_test);
    }
    //
    */



}
void rmCircle::Update(float dt){
    // Update the data (buffer variables) here
}
void rmCircle::Update(ROS_INTERFACE &ros_interface){
    // Update the data (buffer variables) here
}

void rmCircle::Update(ROS_API &ros_api){
    // Update the data (buffer variables) here

    // Update transform
    //--------------------------------//
    if (_frame_id.size() > 0){
        // Get tf
        bool tf_successed = false;
        glm::mat4 _model_tf = ROStf2GLMmatrix(ros_api.get_tf(_frame_id, tf_successed));
        set_pose_modle_ref_by_world(_model_tf);
        // end Get tf
    }else{
        if ( ros_api.ros_interface.is_topic_got_frame(_ROS_topic_id) ){
            // Get tf
            bool tf_successed = false;
            glm::mat4 _model_tf = ROStf2GLMmatrix(ros_api.get_tf(_ROS_topic_id, tf_successed));
            set_pose_modle_ref_by_world(_model_tf);
            // end Get tf
        }
    }
    //--------------------------------//
    // end Update transform


}


void rmCircle::Render(std::shared_ptr<ViewManager> &_camera_ptr){

    glBindVertexArray(m_shape.vao);

	_program_ptr->UseProgram();
    // The transformation matrices and projection matrices
    glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, value_ptr( get_mv_matrix(_camera_ptr, m_shape.model) ));
    glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, value_ptr(_camera_ptr->GetProjectionMatrix()));

    // Setting
    glLineWidth(_line_width);
    // Draw the element according to ebo
    // glDrawElements(GL_TRIANGLES, m_shape.indexCount, GL_UNSIGNED_INT, 0);
    glDrawArrays(GL_POINTS, 0, m_shape.indexCount); // draw part of points
    //--------------------------------//
    glLineWidth(1.0); // default line width
    _program_ptr->CloseProgram();
}




// Insert method for circle
//-------------------------------------//
void rmCircle::insert_circle(const std::vector<circle_data> & data_list_in ){
    long long num_shape = data_list_in.size();
    if (num_shape > _max_num_shape){
        num_shape = _max_num_shape;
    }

    // vao vbo
    glBindVertexArray(m_shape.vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo); // Start to use the buffer


    m_shape.indexCount = num_shape * _num_vertex_per_shape;
    circle_data * vertex_ptr = (circle_data *)glMapBufferRange(GL_ARRAY_BUFFER, 0, num_shape * _num_vertex_per_shape * sizeof(circle_data), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    for (size_t i = 0; i < (num_shape*_num_vertex_per_shape); i++){
        vertex_ptr[i] = data_list_in[i];
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);
}
void rmCircle::insert_circle(const std::map<long long, circle_data> & data_map_in ){
    long long num_shape = data_map_in.size();
    if (num_shape > _max_num_shape){
        num_shape = _max_num_shape;
    }

    // vao vbo
    glBindVertexArray(m_shape.vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo); // Start to use the buffer


    m_shape.indexCount = num_shape * _num_vertex_per_shape;
    circle_data * vertex_ptr = (circle_data *)glMapBufferRange(GL_ARRAY_BUFFER, 0, num_shape * _num_vertex_per_shape * sizeof(circle_data), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    size_t i = 0;
    for (auto it=data_map_in.cbegin(); it != data_map_in.cend(); ++it){
        vertex_ptr[i] = it->second;
        i++;
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);
}
//-------------------------------------//
