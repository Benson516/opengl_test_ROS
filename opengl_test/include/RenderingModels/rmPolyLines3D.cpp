#include "rmPolyLines3D.h"





rmPolyLines3D::rmPolyLines3D(std::string _path_Assets_in){
    _path_Shaders_sub_dir += "Circle/";
    init_paths(_path_Assets_in);
    //
    _num_vertex_per_shape = 1;
    _max_num_shape = 1000;
    _max_num_vertex = _max_num_shape*(long long)(_num_vertex_per_shape);
    //
	Init();
}
void rmPolyLines3D::Init(){
    //
	_program_ptr.reset( new ShaderProgram() );
    // Load shaders
    _program_ptr->AttachShader(get_full_Shader_path("PolyLines3D.vs.glsl"), GL_VERTEX_SHADER);
    _program_ptr->AttachShader(get_full_Shader_path("PolyLines3D.fs.glsl"), GL_FRAGMENT_SHADER);
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
void rmPolyLines3D::LoadModel(){
    glGenVertexArrays(1, &m_shape.vao);
	glBindVertexArray(m_shape.vao);

    glGenBuffers(1, &m_shape.vbo);
	glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo);
    glBufferData(GL_ARRAY_BUFFER, _max_num_vertex * sizeof(point_data), NULL, GL_DYNAMIC_DRAW); // test, change to dynamic draw to assign point cloud
    // Directly assign data to memory of GPU
    //--------------------------------------------//
	point_data * vertex_ptr = (point_data *)glMapBufferRange(GL_ARRAY_BUFFER, 0, _max_num_vertex * sizeof(point_data), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
	size_t _j = 0;
    float radious = 1.0;
	for (size_t i = 0; i < _max_num_shape; i++)
	{
        // Center
		vertex_ptr[_j].position[0] = 2.0*radious*(i/_num_vertex_per_shape);
		vertex_ptr[_j].position[1] = 0.0f;
		vertex_ptr[_j].position[2] = 0.0f;
		vertex_ptr[_j].color[0] = 1.0f; //
		vertex_ptr[_j].color[1] = 1.0f; //
		vertex_ptr[_j].color[2] = 1.0f; //
        _j++;
	}
	glUnmapBuffer(GL_ARRAY_BUFFER);
    //--------------------------------------------//

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(point_data), NULL);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(point_data), (void *)(sizeof(glm::vec3) + sizeof(float) )  );
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);

    m_shape.indexCount = _max_num_vertex;
    //--------------------------------------------//


}
void rmPolyLines3D::Update(float dt){
    // Update the data (buffer variables) here
}
void rmPolyLines3D::Update(ROS_INTERFACE &ros_interface){
    // Update the data (buffer variables) here
}

void rmPolyLines3D::Update(ROS_API &ros_api){
    // Update the data (buffer variables) here
}


void rmPolyLines3D::Render(std::shared_ptr<ViewManager> &_camera_ptr){

    glBindVertexArray(m_shape.vao);

	_program_ptr->UseProgram();
    // The transformation matrices and projection matrices
    glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, value_ptr( get_mv_matrix(_camera_ptr, m_shape.model) ));
    glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, value_ptr(_camera_ptr->GetProjectionMatrix()));

    // Loop for all lines
    for (size_t i=0; i < line_list.size(); ++i){
        _draw_one_poly_line( line_list[i] );
    }

    _program_ptr->CloseProgram();
}


void rmPolyLines3D::_draw_one_poly_line(std::vector<point_data> &a_line_in){
    // Update vbo
    update_GL_data(a_line_in);

    // Setting
    glLineWidth(2.0);
    // Draw the element
    glDrawArrays(GL_LINES, 0, m_shape.indexCount); // draw part of points
    //--------------------------------//
    glLineWidth(1.0); // default line width
}

void rmPolyLines3D::update_GL_data(std::vector<point_data> &a_line_in){
    long long num_shape = a_line_in.size();
    if (num_shape > _max_num_shape){
        num_shape = _max_num_shape;
    }

    // vao vbo
    glBindVertexArray(m_shape.vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo); // Start to use the buffer


    m_shape.indexCount = num_shape * _num_vertex_per_shape;
    point_data * vertex_ptr = (point_data *)glMapBufferRange(GL_ARRAY_BUFFER, 0, num_shape * _num_vertex_per_shape * sizeof(point_data), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    size_t _j = 0;
    for (size_t i = 0; i < (num_shape*_num_vertex_per_shape); i++){
        //
        vertex_ptr[i] = a_line_in[i];
    }
    glUnmapBuffer(GL_ARRAY_BUFFER);
}


// Insert method for circle
//-------------------------------------//
void rmPolyLines3D::push_back_a_line(const vector<point_data> & a_line_in ){
    line_list.push_back(a_line_in);
}
//-------------------------------------//
