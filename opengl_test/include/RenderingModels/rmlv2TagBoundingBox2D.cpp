#include "rmlv2TagBoundingBox2D.h"







rmlv2TagBoundingBox2D::rmlv2TagBoundingBox2D(
    std::string _path_Assets_in,
    int _ROS_topic_id_in,
    bool is_perspected_in,
    bool is_moveable_in
):
    is_perspected(is_perspected_in),
    is_moveable(is_moveable_in),
    _ROS_topic_id(_ROS_topic_id_in),
    //
    rm_box(_path_Assets_in, _ROS_topic_id_in, is_perspected_in, is_moveable_in),
    rm_text(_path_Assets_in)
{
    //
	Init();
}
void rmlv2TagBoundingBox2D::Init(){





}

void rmlv2TagBoundingBox2D::Update(float dt){
    // Update the data (buffer variables) here
}
void rmlv2TagBoundingBox2D::Update(ROS_INTERFACE &ros_interface){
    // Update the data (buffer variables) here
}

void rmlv2TagBoundingBox2D::Update(ROS_API &ros_api){
    // Update the data (buffer variables) here
    // test, use transform
    ros::Time msg_time;
    bool _result = false;

    _result = ros_api.get_message(_ROS_topic_id, msg_out_ptr, msg_time);




    // Move in 3D space
    if ( is_perspected && ros_api.ros_interface.is_topic_got_frame(_ROS_topic_id)){
        // Get tf
        bool tf_successed = false;
        glm::mat4 _model_tf = ROStf2GLMmatrix(ros_api.get_tf(_ROS_topic_id, tf_successed));
        set_pose_modle_ref_by_world(_model_tf);
        // end Get tf
    }

    if (_result){
        // update_GL_data();
        // rm_text.insert_text();
    }

    //
    rm_box.Update(ros_api);

}


void rmlv2TagBoundingBox2D::Render(std::shared_ptr<ViewManager> &_camera_ptr){

    rm_box.Render(_camera_ptr);
    rm_text.Render(_camera_ptr);
}
