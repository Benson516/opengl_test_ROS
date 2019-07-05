#include "rmlv2TagBoundingBox3D.h"




rmlv2TagBoundingBox3D::rmlv2TagBoundingBox3D(
    std::string _path_Assets_in,
    int _ROS_topic_id_in
):
    _ROS_topic_id(_ROS_topic_id_in),
    //
    rm_box(_path_Assets_in, _ROS_topic_id_in),
    rm_text(_path_Assets_in, _ROS_topic_id_in)
{
    //
	Init();
}
void rmlv2TagBoundingBox3D::Init(){


    // For adjusting the model pose by public methods
    attach_pose_model_by_model_ref_ptr( *rm_box.get_model_m_ptr() );
    attach_pose_model_by_model_ref_ptr( *rm_text.get_model_m_ptr() );

}

void rmlv2TagBoundingBox3D::Update(float dt){
    // Update the data (buffer variables) here
}
void rmlv2TagBoundingBox3D::Update(ROS_INTERFACE &ros_interface){
    // Update the data (buffer variables) here
}

void rmlv2TagBoundingBox3D::Update(ROS_API &ros_api){
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
    rm_box.Update(ros_api);
    rm_text.Update(ros_api);
}


void rmlv2TagBoundingBox3D::Render(std::shared_ptr<ViewManager> &_camera_ptr){

    rm_box.Render(_camera_ptr);
    rm_text.Render(_camera_ptr);
}



void rmlv2TagBoundingBox3D::update_GL_data(){
    // Reset
    text_list.clear();
    //
    if (msg_out_ptr->lidRoiBox.size() == 0){
        // Insert texts
        rm_text.insert_text(text_list);
        return;
    }
    long long num_box = msg_out_ptr->lidRoiBox.size();
    /*
    if (num_box > _max_num_box){
        num_box = _max_num_box;
    }
    */

    auto * _point_ptr = &(msg_out_ptr->lidRoiBox[0].p0);
    size_t _j = 0;
    for (size_t i = 0; i < num_box; i++)
    {

        /*
        _point_ptr = &(msg_out_ptr->lidRoiBox[i].p0);
        vertex_ptr[_j].position[0] = (_point_ptr)->x;
        vertex_ptr[_j].position[1] = (_point_ptr)->y;
        vertex_ptr[_j].position[2] = (_point_ptr )->z;
        vertex_ptr[_j].color[0] = 1.0f; // If we don't keep udating the color, the color will be lost when resizing the window.
        vertex_ptr[_j].color[1] = 1.0f;
        vertex_ptr[_j].color[2] = 1.0f;
        _j++;
        */

        _point_ptr = &(msg_out_ptr->lidRoiBox[i].p0);
        //
        text_list.emplace_back(
            "#" + std::to_string(i),
            glm::vec3(_point_ptr->x, _point_ptr->y, _point_ptr->z),
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
