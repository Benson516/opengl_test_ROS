#ifndef RM_TEXT_2D_H
#define RM_TEXT_2D_H

#include "rmBaseModel.h"






class rmText2D : public rmBaseModel
{
public:
    rmText2D();
    //
	void Update(float dt);
    void Update(ROS_INTERFACE &ros_interface);
    void Update(ROS_API &ros_api);
	void Render(std::shared_ptr<ViewManager> _camera_ptr);

protected:
    void Init();
    virtual void LoadModel();
    //
    // int _ROS_topic_id;
    // std::shared_ptr< msgs::LidRoi > box3d_out_ptr;
    // ros::Time msg_time;

    void selectFont2D(int newfont);
    void text2D_output(float x, float y, std::string string_in);

private:



    std::string text_current;


};

#endif // RM_TEXT_2D_H
