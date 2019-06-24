#include "rmImageBoard.h"

static const GLfloat window_positions[] =
{
    // Position i, texcord i
	1.0f,-1.0f,1.0f,0.0f,  // right-down
	-1.0f,-1.0f,0.0f,0.0f, // left-down
	-1.0f,1.0f,0.0f,1.0f,  // left-up
	1.0f,1.0f,1.0f,1.0f    // right-up
};


rmImageBoard::rmImageBoard(
    std::string _path_Assets_in,
    std::string image_file_in,
    bool is_perspected_in,
    bool is_moveable_in,
    bool is_color_transformed_in
):
    is_perspected(is_perspected_in),
    is_moveable(is_moveable_in),
    is_color_transformed(is_color_transformed_in),
    fps_of_update(image_file_in)
{
    _path_Shaders_sub_dir += "ImageBoard/";
    init_paths(_path_Assets_in);
    textName = image_file_in;
    //
    is_dynamically_updated = false;
    //
	Init();
}
rmImageBoard::rmImageBoard(
    std::string _path_Assets_in,
    int _ROS_topic_id_in,
    bool is_perspected_in,
    bool is_moveable_in,
    bool is_color_transformed_in
):
    is_perspected(is_perspected_in),
    is_moveable(is_moveable_in),
    is_color_transformed(is_color_transformed_in),
    _ROS_topic_id(_ROS_topic_id_in),
    fps_of_update( std::string("Image ") + std::to_string(_ROS_topic_id_in) )
{
    _path_Shaders_sub_dir += "ImageBoard/";
    init_paths(_path_Assets_in);
    //
    is_dynamically_updated = true;
    //
	Init();
}
void rmImageBoard::Init(){
    //
	_program_ptr.reset( new ShaderProgram() );
    // Load shaders
    //----------------------------------------//
    // VS
    if (is_perspected){
        _program_ptr->AttachShader(get_full_Shader_path("ImageBoard.vs.Perspected.glsl"), GL_VERTEX_SHADER);
    }else{
        if (is_moveable){
            _program_ptr->AttachShader(get_full_Shader_path("ImageBoard.vs.Moveable.glsl"), GL_VERTEX_SHADER);
        }else{ // Background
            _program_ptr->AttachShader(get_full_Shader_path("ImageBoard.vs.Background.glsl"), GL_VERTEX_SHADER);
        }
    }
    // FS
    if (is_color_transformed){
        _program_ptr->AttachShader(get_full_Shader_path("ImageBoard.fs.ColorTransformedAlpha.glsl"), GL_FRAGMENT_SHADER);
    }else{
        _program_ptr->AttachShader(get_full_Shader_path("ImageBoard.fs.Alpha.glsl"), GL_FRAGMENT_SHADER);
    }
    //----------------------------------------//
    // Link _program_ptr
	_program_ptr->LinkProgram();
    //

    // Initialize variables
    // Init model matrices
	m_shape.model = glm::mat4(1.0);
    attach_pose_model_by_model_ref_ptr(m_shape.model); // For adjusting the model pose by public methods
    // Colors
    _alpha = 0.7;
    _color_transform = glm::vec4(1.0f);
    //

    // Cache uniform variable id
    //----------------------------------------//
    if (is_perspected){
        uniforms.proj_matrix = glGetUniformLocation(_program_ptr->GetID(), "proj_matrix");
        uniforms.mv_matrix = glGetUniformLocation(_program_ptr->GetID(), "mv_matrix");
    }else if (is_moveable){
        uniforms.mv_matrix = glGetUniformLocation(_program_ptr->GetID(), "mv_matrix");
    }
    if (is_color_transformed){
            uniforms.color_transform = glGetUniformLocation(_program_ptr->GetID(), "color_transform");
    }
	uniforms.alpha = glGetUniformLocation(_program_ptr->GetID(), "alpha");
    //----------------------------------------//


    //Load model to shader _program_ptr
	LoadModel();

}
void rmImageBoard::LoadModel(){
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
    glGenTextures(1, &m_shape.m_texture);
	glBindTexture(GL_TEXTURE_2D, m_shape.m_texture);
    if (textName != ""){
        //Load texture data from file
        std::cout << "start loading <" << textName << ">\n";
    	TextureData tdata = Common::Load_png(get_full_Assets_path(textName).c_str());
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, tdata.width, tdata.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, tdata.data);
    }
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    std::cout << "Load texture success!\n";

}
void rmImageBoard::Update(float dt){
    // Update the data (buffer variables) here
}
void rmImageBoard::Update(ROS_INTERFACE &ros_interface){
    // Update the data (buffer variables) here
    // Check if this image needs to be updated.
    if (!is_dynamically_updated){
        return;
    }

    bool _result = ros_interface.get_Image( _ROS_topic_id, msg_out_ptr);


    if (_result){
        // evaluation
        TIME_STAMP::Period period_image(fps_of_update.name);
        //
        update_GL_data();
        // evaluation
        period_image.stamp();  period_image.show_usec();
        // FPS
        fps_of_update.stamp();  fps_of_update.show();
    }

    // Move in 3D space
    if (is_perspected && ros_interface.is_topic_got_frame(_ROS_topic_id)){
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
void rmImageBoard::Update(ROS_API &ros_api){
    // Update the data (buffer variables) here
    // Check if this image needs to be updated.
    if (!is_dynamically_updated){
        return;
    }

    bool _result = false;
    // Scops for any_ptr
    {
        boost::any any_ptr;
        _result = ros_api.get_any_message( _ROS_topic_id, any_ptr );
        if (_result){
            std::shared_ptr< cv::Mat > *_ptr_ptr = boost::any_cast< std::shared_ptr< cv::Mat > >( &any_ptr );
            msg_out_ptr = *_ptr_ptr;
        }
    }// end Scops for any_ptr

    if (_result){
        // evaluation
        // TIME_STAMP::Period period_image(fps_of_update.name);
        //
        update_GL_data();
        // evaluation
        // period_image.stamp();  period_image.show_usec();
        // FPS
        // fps_of_update.stamp();  fps_of_update.show();
    }


    // Move in 3D space
    if (is_perspected && ros_api.ros_interface.is_topic_got_frame(_ROS_topic_id)){
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


void rmImageBoard::Render(std::shared_ptr<ViewManager> _camera_ptr){

    glBindVertexArray(m_shape.vao);
	_program_ptr->UseProgram();

    if (is_perspected){
        //
        // m_shape.model = translateMatrix * rotateMatrix * scaleMatrix;
        // The transformation matrices and projection matrices
        glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, value_ptr( get_mv_matrix(_camera_ptr, m_shape.model) ));
        glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, value_ptr(_camera_ptr->GetProjectionMatrix()));
    }else{
        if (is_moveable){
            // Note: the rotation is mainly for z-axis rotation
            // Note 2: The tranalation/rotation/scale is based on the "center" of the image
            // m_shape.model = translateMatrix * rotateMatrix * scaleMatrix;
            glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, value_ptr( m_shape.model ));
        }else{
            // background
            // Nothing, for saving computation
        }
    }
    //
    glUniform1f(uniforms.alpha, _alpha); // The alpha, if alpha < 0.0 then it's disabled
    if (is_color_transformed){
        // Color transform will resulted in feelable delay in display.
        glUniform4fv(uniforms.color_transform, 1, value_ptr(_color_transform) );
    }

    // glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_shape.m_texture);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    //
    _program_ptr->CloseProgram();
}




void rmImageBoard::update_GL_data(){
    //
    m_shape.width = msg_out_ptr->cols;
    m_shape.height = msg_out_ptr->rows;

    // vao vbo
    glBindVertexArray(m_shape.vao);
    // glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo); // Start to use the buffer

    // Texture
    glBindTexture(GL_TEXTURE_2D, m_shape.m_texture);
    cv::Mat image_in = *msg_out_ptr;
    //use fast 4-byte alignment (default anyway) if possible
    glPixelStorei(GL_UNPACK_ALIGNMENT, (image_in.step & 3) ? 1 : 4);
    //set length of one complete row in data (doesn't need to equal image.cols)
    glPixelStorei(GL_UNPACK_ROW_LENGTH, image_in.step/image_in.elemSize());
    //
    cv::Mat flipped_image;
    cv::flip(image_in, flipped_image, 0);
    // glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, flipped_image.width, flipped_image.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, flipped_image.data);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, flipped_image.cols, flipped_image.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, flipped_image.data);
    //
}
