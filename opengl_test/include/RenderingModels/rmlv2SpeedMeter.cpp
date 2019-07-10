#include "rmlv2SpeedMeter.h"




rmlv2SpeedMeter::rmlv2SpeedMeter(
    std::string _path_Assets_in,
    int _ROS_topic_id_in,
    std::string frame_id_in,
    glm::vec4 color_vec4_in,
    bool is_perspected_in,
    bool is_moveable_in
):
    _ROS_topic_id(_ROS_topic_id_in),
    //
    rm_board(_path_Assets_in, frame_id_in, color_vec4_in, is_perspected_in, is_moveable_in),
    rm_text(_path_Assets_in, _ROS_topic_id_in)
{
    //
	Init();
}
void rmlv2SpeedMeter::Init(){

    // For adjusting the model pose by public methods
    attach_pose_model_by_model_ref_ptr( *rm_board.get_model_m_ptr() );
    attach_pose_model_by_model_ref_ptr( *rm_text.get_model_m_ptr() );


    rm_board.shape.setBoardSizePixel(200, 100);
    rm_board.shape.setBoardPositionCVPixel(5,5,1,ALIGN_X::RIGHT, ALIGN_Y::TOP);
    rm_text.shape = rm_board.shape;
}

void rmlv2SpeedMeter::Update(float dt){
    // Update the data (buffer variables) here
}
void rmlv2SpeedMeter::Update(ROS_INTERFACE &ros_interface){
    // Update the data (buffer variables) here
}

void rmlv2SpeedMeter::Update(ROS_API &ros_api){
    // Update the data (buffer variables) here
    // test, use transform
    ros::Time msg_time;
    bool _result = false;
    _result = ros_api.get_message(_ROS_topic_id, msg_out_ptr, msg_time);

    if (_result){
        update_GL_data();
        // rm_text.insert_text();
    }

    //
    rm_board.Update(ros_api);
    rm_text.Update(ros_api);
}


void rmlv2SpeedMeter::Render(std::shared_ptr<ViewManager> &_camera_ptr){

    rm_board.Render(_camera_ptr);
    rm_text.Render(_camera_ptr);
}


void rmlv2SpeedMeter::update_GL_data(){
    // Reset
    text2D_flat_list.clear();


    //
    std::cout << "Speed (km/h): " << (msg_out_ptr->ego_speed)*3.6 << ", ";
    std::cout << "yaw_rate (deg/s): " << (msg_out_ptr->yaw_rate) << "\n";

    std::string _str_out( std::to_string( int((msg_out_ptr->ego_speed)*3.6) ) + " km/r" );

    text2D_flat_list.emplace_back(
        _str_out,
        glm::vec2(0.0f),
        48,
        glm::vec3(1.0f, 0.5f, 0.0f),
        ALIGN_X::RIGHT,
        ALIGN_Y::BUTTON,
        0,
        0,
        false, // relative to board
        false
    );


    // Insert texts
    rm_text.insert_text( text2D_flat_list );

}
