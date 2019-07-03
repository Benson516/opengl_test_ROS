#include "rmlv2TagBoundingBox2D.h"







rmlv2TagBoundingBox2D::rmlv2TagBoundingBox2D(
    std::string _path_Assets_in,
    int _ROS_topic_id_in,
    bool is_perspected_in,
    bool is_moveable_in
):
    is_perspected(is_perspected_in),
    is_moveable(is_moveable_in),
    _ROS_topic_id(_ROS_topic_id_in)
{
    _path_Shaders_sub_dir += "BoundingBox2D/";
    init_paths(_path_Assets_in);
    //
    _num_vertex_idx_per_box = 4*2; // 6*(3*2);
    _num_vertex_per_box = 4; // 8;
    _max_num_box = 1000;
    _max_num_vertex_idx = _max_num_box*(long long)(_num_vertex_idx_per_box);
    _max_num_vertex = _max_num_box*(long long)(_num_vertex_per_box);
    //
	Init();
}
void rmlv2TagBoundingBox2D::Init(){



    //Load model to shader _program_ptr
	LoadModel();

}
void rmlv2TagBoundingBox2D::LoadModel(){



}
void rmlv2TagBoundingBox2D::Update(float dt){
    // Update the data (buffer variables) here
}
void rmlv2TagBoundingBox2D::Update(ROS_INTERFACE &ros_interface){
    // Update the data (buffer variables) here

    // test, use transform
    ros::Time msg_time;
    bool _result = ros_interface.get_void_message( _ROS_topic_id, &msg_out_ptr, msg_time);

    if (_result){
        // update_GL_data();
    }

    // Move in 3D space
    if (  is_perspected && ros_interface.is_topic_got_frame(_ROS_topic_id)){
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

void rmlv2TagBoundingBox2D::Update(ROS_API &ros_api){
    // Update the data (buffer variables) here
    // test, use transform
    ros::Time msg_time;
    bool _result = false;

    _result = ros_api.get_message(_ROS_topic_id, msg_out_ptr, msg_time);

    if (_result){
        // update_GL_data();
    }


    // Move in 3D space
    if ( is_perspected && ros_api.ros_interface.is_topic_got_frame(_ROS_topic_id)){
        // Get tf
        bool tf_successed = false;
        glm::mat4 _model_tf = ROStf2GLMmatrix(ros_api.get_tf(_ROS_topic_id, tf_successed));
        set_pose_modle_ref_by_world(_model_tf);
        // end Get tf
    }
}


void rmlv2TagBoundingBox2D::Render(std::shared_ptr<ViewManager> _camera_ptr){


}
