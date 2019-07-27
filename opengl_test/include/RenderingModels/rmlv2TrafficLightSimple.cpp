#include "rmlv2TrafficLightSimple.h"




rmlv2TrafficLightSimple::rmlv2TrafficLightSimple(
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
void rmlv2TrafficLightSimple::Init(){

    // For adjusting the model pose by public methods
    attach_pose_model_by_model_ref_ptr( *rm_board.get_model_m_ptr() );
    attach_pose_model_by_model_ref_ptr( *rm_text.get_model_m_ptr() );


    rm_board.shape.setBoardSizePixel(280, 80);
    rm_board.shape.setBoardPositionCVPixel(-5,95,1,ALIGN_X::RIGHT, ALIGN_Y::TOP);
    rm_text.shape = rm_board.shape;


    // Reset
    text2D_flat_list.clear();

}

void rmlv2TrafficLightSimple::Update(float dt){
    // Update the data (buffer variables) here
}
void rmlv2TrafficLightSimple::Update(ROS_INTERFACE &ros_interface){
    // Update the data (buffer variables) here
}

void rmlv2TrafficLightSimple::Update(ROS_API &ros_api){
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


void rmlv2TrafficLightSimple::Render(std::shared_ptr<ViewManager> &_camera_ptr){

    rm_board.Render(_camera_ptr);
    rm_text.Render(_camera_ptr);
}

void rmlv2TrafficLightSimple::Reshape(const glm::ivec2 & viewport_size_in){
    _viewport_size = viewport_size_in;
    // updateBoardGeo();
    rm_board.Reshape(viewport_size_in);
    rm_text.shape = rm_board.shape;
    rm_text.Reshape(viewport_size_in);
}

void rmlv2TrafficLightSimple::update_GL_data(){
    // Reset
    text2D_flat_list.clear();


    //
    // std::cout << "Speed (km/h): " << (msg_out_ptr->ego_speed)*3.6 << ", ";
    // std::cout << "yaw_rate (deg/s): " << (msg_out_ptr->yaw_rate) << "\n";



    int light_status = int(msg_out_ptr->Dspace_Flag02);
    int light_CD     = int(msg_out_ptr->Dspace_Flag03);  // Count-down time

    std::string _str_out( "Traffic Light      \n" );
    glm::vec3 _text_color(0.0f);

    switch (light_status){
        case 0: // red
            _str_out += "Red:   ";
            _text_color = glm::vec3(1.0f, 0.0f, 0.0f);
            break;
        case 1: // yello
            _str_out += "Yello: ";
            _text_color = glm::vec3(0.897f, 0.837f, 0.0f);
            break;
        case 2: // green
            _str_out += "Green: ";
            _text_color = glm::vec3(0.0f, 0.886f, 0.046f);
            break;
        default:
            _str_out = "--";
            break;
    }
    //
    if ( light_CD < 10){
        _str_out += " ";
    }
    _str_out += std::to_string(light_CD) + " sec.";


    text2D_flat_list.emplace_back(
        _str_out,
        glm::vec2(float(rm_board.shape.board_width-15), float(rm_board.shape.board_height/2.0f)),
        // glm::vec2( 0.0f, 0.0f),
        36,
        _text_color,
        ALIGN_X::RIGHT,
        ALIGN_Y::CENTER,
        0,
        0,
        false,
        false
    );


    // Insert texts
    rm_text.insert_text( text2D_flat_list );

}
