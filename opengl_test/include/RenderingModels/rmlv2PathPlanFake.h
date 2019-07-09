#ifndef RM_LV2_PATH_PLAN_FAKE_H
#define RM_LV2_PATH_PLAN_FAKE_H

#include "rmBaseModel.h"

//
#include "rmSweepingObject.h"
#include "rmText3D_v2.h"


class rmlv2PathPlanFake : public rmBaseModel
{
public:
    rmlv2PathPlanFake(
        std::string _path_Assets_in,
        int _ROS_topic_id_in
    );
    //
	void Update(float dt);
    void Update(ROS_INTERFACE &ros_interface);
    void Update(ROS_API &ros_api);
	void Render(std::shared_ptr<ViewManager> &_camera_ptr);


protected:
    void Init();
    //
    int _ROS_topic_id;
    // std::shared_ptr< msgs::LidRoi > msg_out_ptr;
    std::shared_ptr< msgs::VehInfo  > msg_out_ptr;
    // ros::Time msg_time;

    //
    rmSweepingObject rm_path;
    rmText3D_v2 rm_text;

    void update_GL_data();

    void get_pose2D_sim(const glm::vec3 &pose2D_0, const glm::vec2 &twist2D, double dT, glm::vec3 &pose2D_out);

    // Param
    float _sim_time; // sec.
    glm::vec2 _granularity; // 20 cm, 5 deg.
    int _max_sim_point;

private:
    std::vector<glm::vec3> section_vertexes;
    std::vector<glm::vec3> _path;
    std::vector<rmText3D_v2::text_billboard_data> text_list;

};

#endif // RM_LV2_PATH_PLAN_FAKE_H
