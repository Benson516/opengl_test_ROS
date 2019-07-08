#ifndef RM_LV2_OBJECT_TRACKING_H
#define RM_LV2_OBJECT_TRACKING_H

#include "rmBaseModel.h"

//
#include "rmCircle.h"
#include "rmText3D_v2.h"


class rmlv2ObjectTracking : public rmBaseModel
{
public:
    rmlv2ObjectTracking(
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
    std::shared_ptr< msgs::DetectedObjectArray > msg_out_ptr;
    // ros::Time msg_time;

    //
    rmLidarBoundingBox rm_box;
    rmText3D_v2 rm_text;

    void update_GL_data();

private:

    std::vector<rmText3D_v2::text_billboard_data> text_list;

};

#endif // RM_LV2_OBJECT_TRACKING_H
