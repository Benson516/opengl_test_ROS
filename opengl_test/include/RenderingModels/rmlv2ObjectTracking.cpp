#include "rmlv2ObjectTracking.h"




rmlv2ObjectTracking::rmlv2ObjectTracking(
    std::string _path_Assets_in,
    int _ROS_topic_id_in
):
    _ROS_topic_id(_ROS_topic_id_in),
    //
    rm_circle(_path_Assets_in),
    rm_text(_path_Assets_in, _ROS_topic_id_in)
{
    // init_paths(_path_Assets_in);
    //
	Init();
}
void rmlv2ObjectTracking::Init(){


    // For adjusting the model pose by public methods
    attach_pose_model_by_model_ref_ptr( *rm_circle.get_model_m_ptr() );
    attach_pose_model_by_model_ref_ptr( *rm_text.get_model_m_ptr() );


    //Load model to shader _program_ptr
	LoadModel();

}

void rmlv2ObjectTracking::LoadModel(){

}

void rmlv2ObjectTracking::Update(float dt){
    // Update the data (buffer variables) here
}
void rmlv2ObjectTracking::Update(ROS_INTERFACE &ros_interface){
    // Update the data (buffer variables) here
}

void rmlv2ObjectTracking::Update(ROS_API &ros_api){
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
    rm_circle.Update(ros_api);
    rm_text.Update(ros_api);
}


void rmlv2ObjectTracking::Render(std::shared_ptr<ViewManager> &_camera_ptr){

    rm_circle.Render(_camera_ptr);
    rm_text.Render(_camera_ptr);
}



void rmlv2ObjectTracking::update_GL_data(){
    // Reset
    text_list.clear();
    //
    if (msg_out_ptr->objects.size() == 0){
        // Insert texts
        rm_text.insert_text(text_list);
        return;
    }
    long long num_box = msg_out_ptr->objects.size();
    // if (num_box > _max_num_box){
    //     num_box = _max_num_box;
    // }

    auto * _point_1_ptr = &(msg_out_ptr->objects[0].bPoint.p0);
    auto * _point_2_ptr = &(msg_out_ptr->objects[0].bPoint.p0);
    size_t _j = 0;
    for (size_t i = 0; i < num_box; i++)
    {
        std::string _s_tag( "#" + std::to_string( msg_out_ptr->objects[i].camInfo.id ) );

        // _point_1_ptr = &(msg_out_ptr->objects[i].bPoint.p0);
        // text_list.emplace_back(
        //     _s_tag,
        //     glm::vec3(_point_1_ptr->x, _point_1_ptr->y, _point_1_ptr->z),
        //     glm::vec2(0.0f),
        //     1.0f,
        //     glm::vec3(0.0f, 1.0f, 0.0f),
        //     rmText3D_v2::ALIGN_X::LEFT,
        //     rmText3D_v2::ALIGN_Y::BUTTON
        // );
        _point_1_ptr = &(msg_out_ptr->objects[i].bPoint.p1);
        _point_2_ptr = &(msg_out_ptr->objects[i].bPoint.p6);
        text_list.emplace_back(
            _s_tag,
            0.5f*(glm::vec3(_point_1_ptr->x, _point_1_ptr->y, _point_1_ptr->z) + glm::vec3(_point_2_ptr->x, _point_2_ptr->y, _point_2_ptr->z)) + glm::vec3(0.0f, 0.0f, 0.5f),
            glm::vec2(0.0f),
            1.0f,
            glm::vec3(0.0f, 1.0f, 0.0f),
            rmText3D_v2::ALIGN_X::CENTER,
            rmText3D_v2::ALIGN_Y::BUTTON
        );
    }


    // Insert texts
    rm_text.insert_text(text_list);

}
