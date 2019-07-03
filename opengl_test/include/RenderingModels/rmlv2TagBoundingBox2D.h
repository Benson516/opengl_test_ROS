#ifndef RM_LV2_TAG_BOUNDINGBOX_2D_H
#define RM_LV2_TAG_BOUNDINGBOX_2D_H

#include "rmBaseModel.h"

//
#include "rmBoundingBox2D.h"
#include "rmText3D_v2.h"


class rmlv2TagBoundingBox2D : public rmBaseModel
{
public:
    rmlv2TagBoundingBox2D(
        std::string _path_Assets_in,
        int _ROS_topic_id_in,
        bool is_perspected_in=true,
        bool is_moveable_in=true
    );
    //
	void Update(float dt);
    void Update(ROS_INTERFACE &ros_interface);
    void Update(ROS_API &ros_api);
	void Render(std::shared_ptr<ViewManager> _camera_ptr);

    void setup_params(int im_width_in, int im_height_in, int image_offset_in_box_cv_x_in, int image_offset_in_box_cv_y_in){

    }

protected:
    void Init();
    //
    int _ROS_topic_id;
    std::shared_ptr< msgs::CamObj > msg_out_ptr;
    // ros::Time msg_time;

    // Settings
    bool is_perspected;
    bool is_moveable;

    //
    rmBoundingBox2D rm_box;
    rmText3D_v2 rm_text;

private:






};

#endif // RM_LV2_TAG_BOUNDINGBOX_2D_H
