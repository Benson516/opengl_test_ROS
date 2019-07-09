#include "rmlv2PathPlanFake.h"


#include <math.h>       // ceil


rmlv2PathPlanFake::rmlv2PathPlanFake(
    std::string _path_Assets_in,
    int _ROS_topic_id_in
):
    _ROS_topic_id(_ROS_topic_id_in),
    //
    rm_path(_path_Assets_in, _ROS_topic_id_in),
    rm_text(_path_Assets_in, _ROS_topic_id_in)
{
    //
	Init();
}
void rmlv2PathPlanFake::Init(){

    // For adjusting the model pose by public methods
    attach_pose_model_by_model_ref_ptr( *rm_path.get_model_m_ptr() );
    attach_pose_model_by_model_ref_ptr( *rm_text.get_model_m_ptr() );

}

void rmlv2PathPlanFake::Update(float dt){
    // Update the data (buffer variables) here
}
void rmlv2PathPlanFake::Update(ROS_INTERFACE &ros_interface){
    // Update the data (buffer variables) here
}

void rmlv2PathPlanFake::Update(ROS_API &ros_api){
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
    rm_path.Update(ros_api);
    rm_text.Update(ros_api);
}


void rmlv2PathPlanFake::Render(std::shared_ptr<ViewManager> &_camera_ptr){

    rm_path.Render(_camera_ptr);
    rm_text.Render(_camera_ptr);
}

void rmlv2PathPlanFake::update_GL_data(){

    glm::vec3 pose2D_0(-12.0f, 0.0f, 0.0f);
    glm::vec2 twist2D( msg_out_ptr->ego_speed, msg_out_ptr->yaw_rate*0.017453292519943295 );

    float _sim_time = 3.0f; // sec.
    glm::vec2 _granularity(0.1f, 0.017f); // 10 cm, 1 deg.

    //
    in num_steps = 0;
    float dT = _sim_time;
    //
    int idx_primary_axis = 0;
    glm::vec2 cross_freq = glm::abs(_twist2D/_granularity);
    // Find the primary axis (between translation and rotation)
    if (cross_freq.x >= cross_freq.y){
        idx_primary_axis = 0;
    }else{
        idx_primary_axis = 1;
    }
    if (cross_freq[idx_primary_axis] <= 0.0){
        // Not moving
        dT = _sim_time;
        num_steps = 2;
    } else{
        // else
        dT = 1.0/cross_freq[idx_primary_axis]; // Positive value
        num_steps = int( ceil(self.sim_time/dT) );
    }

    // Calculate path
    _path.clear();
    _path.push_back(pose2D_0);
    glm::vec3 pose2D_path;
    for (size_t i=1; i <= num_steps; ++i ){
        float sim_T = i*dT;
        get_pose2D_sim(pose2D_0, twist2D, sim_T, pose2D_path);
        _path.push_back(pose2D_path);
    }

    insert_curve_Points(_path);

    // // Reset
    // text_list.clear();
    //
    // // Insert texts
    // rm_text.insert_text(text_list);

}



//
void rmlv2PathPlanFake::get_pose2D_sim(const glm::vec3 &pose2D_0, const glm::vec2 &twist2D, double dT, glm::vec3 &pose2D_out){
    double dtheta = twist2D[1]*dT;
    //
    // double x_0 = pose2D_0.x;
    // double y_0 = pose2D_0.y;
    // double theta_0 = pose2D_0.z;
    // double vel_1 = twist2D.x;
    // double omega_1 = twist2D.y;
    //
    // double x_1, y_1, theta_1, r_1;
    //
    if (std::abs(dtheta) < 0.017453292519943295){ // 1 deg., small angle
        pose2D_out[2] = pose2D_0[2] + dtheta;
        pose2D_out.x = pose2D_0.x + twist2D[0]*dT*std::cos( 0.5*(pose2D_out[2] + pose2D_0[2]) );
        pose2D_out.y = pose2D_0.y + twist2D[0]*dT*std::sin( 0.5*(pose2D_out[2] + pose2D_0[2]) );
    }else{ //
        pose2D_out[2] = pose2D_0[2] + dtheta;
        double r_1 = twist2D[0]/twist2D[1];
        pose2D_out.x = pose2D_0.x + r_1*( std::sin(pose2D_out[2]) - std::sin(pose2D_0[2]) );
        pose2D_out.y = pose2D_0.y - r_1*( std::cos(pose2D_out[2]) - std::cos(pose2D_0[2]) );
    }
}
