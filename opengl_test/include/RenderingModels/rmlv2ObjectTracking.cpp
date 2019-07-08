#include "rmlv2ObjectTracking.h"




rmlv2ObjectTracking::rmlv2ObjectTracking(
    std::string _path_Assets_in,
    int _ROS_topic_id_in
):
    _ROS_topic_id(_ROS_topic_id_in),
    //
    rm_polylines3D(_path_Assets_in, _ROS_topic_id_in),
    rm_circle(_path_Assets_in, _ROS_topic_id_in),
    rm_text(_path_Assets_in, _ROS_topic_id_in)
{
    // init_paths(_path_Assets_in);
    //
	Init();
}
void rmlv2ObjectTracking::Init(){


    // For adjusting the model pose by public methods
    attach_pose_model_by_model_ref_ptr( *rm_polylines3D.get_model_m_ptr() );
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
    rm_polylines3D.Update(ros_api);
    rm_circle.Update(ros_api);
    rm_text.Update(ros_api);
}


void rmlv2ObjectTracking::Render(std::shared_ptr<ViewManager> &_camera_ptr){

    rm_polylines3D.Render(_camera_ptr);
    rm_circle.Render(_camera_ptr);
    rm_text.Render(_camera_ptr);
}



void rmlv2ObjectTracking::update_GL_data(){
    // Reset
    rm_polylines3D.reset_line_list();
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

    // Initialize
    std::vector<rmCircle::circle_data> circle_list;

    auto * _point_1_ptr = &(msg_out_ptr->objects[0].bPoint.p0);
    auto * _point_2_ptr = &(msg_out_ptr->objects[0].bPoint.p0);
    size_t _j = 0;
    for (size_t i = 0; i < num_box; i++)
    {
        int obj_id = msg_out_ptr->objects[i].camInfo.id;
        _point_1_ptr = &(msg_out_ptr->objects[i].bPoint.p0);
        _point_2_ptr = &(msg_out_ptr->objects[i].bPoint.p7);
        glm::vec3 point_pose = 0.5f*(glm::vec3(_point_1_ptr->x, _point_1_ptr->y, _point_1_ptr->z) + glm::vec3(_point_2_ptr->x, _point_2_ptr->y, _point_2_ptr->z)) + glm::vec3(0.0f, 0.0f, 0.0f);


        // Reset count
        std::map<int,int>::iterator it_1 = obj_miss_count.find(obj_id);
        if (it_1 == obj_miss_count.end()){
            obj_miss_count[obj_id] = 0;
        }else{
            obj_miss_count[obj_id] -= 2; // We will dd this by one later
        }
        //


        // A line
        //-------------------------//
        std::queue<rmPolyLines3D::point_data> &a_line_queue = line_map[obj_id];
        a_line_queue.emplace(
            point_pose,
            glm::vec3(1.0f, 0.0f, 0.0f)
        );
        while(a_line_queue.size() > 100){ // test, track for 50 points
            a_line_queue.pop();
        }
        rm_polylines3D.push_back_a_line_queue(a_line_queue);
        //-------------------------//

        /*
        // A set of circles
        //-------------------------//
        std::queue<rmPolyLines3D::point_data> point_queue_tmp = a_line_queue;
        while ( !point_queue_tmp.empty() ){
            auto point_tmp = point_queue_tmp.front();
            circle_list.emplace_back(
                point_tmp.position,
                0.5f,
                glm::vec3(1.0f, 0.0f, 0.0f)
            );
            point_queue_tmp.pop();
        }
        //-------------------------//
        */


    }

    // Insert circles
    rm_circle.insert_circle(circle_list);


    // Insert texts
    // rm_text.insert_text(text_list);


    // Clear line
    for (std::map<int,int>::iterator it=obj_miss_count.begin(); it!=obj_miss_count.end(); ++it){
        it->second++;
        if (it->second > 5){ // test, miss count
            // line_map[it->first] = std::queue<rmPolyLines3D::point_data>();
            // it->second = 0;
            line_map.erase(it->first);
            obj_miss_count.erase(it);
        }else if (it->second < 0){
            it->second = 0;
        }
    }
    // std::cout << "line_map.size() = " << line_map.size() << ", ";
    // std::cout << "obj_miss_count.size() = " << obj_miss_count.size() << "\n";


}
