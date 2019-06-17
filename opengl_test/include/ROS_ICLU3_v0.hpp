#ifndef ROS_API_H
#define ROS_API_H

#include <ROS_interface.hpp>

#define __DEBUG__
#define __SUB_IMAGES__
#define __SUB_POINT_CLOUD__

// nickname for topic_id
enum class MSG_ID{
    // tfGeoPoseStamped
    ego_pose_0,
    // Image
#ifdef __SUB_IMAGES__
    camera_0,
    camera_1,
    camera_2,
    camera_3,
    camera_4,
    camera_5,
    camera_6,
    camera_7,
    camera_8,
#endif // __SUB_IMAGES__
    // ITRIPointCloud
#ifdef __SUB_POINT_CLOUD__
    point_cloud_1,
    point_cloud_map,
#endif
    lidar_bounding_box_1,
    // NUM_TOPICS
};



class ROS_API{
public:
    // the ROS_interface
    ROS_INTERFACE ros_interface;
    std::string path_pkg;


    // Data validation (only be used after calling update)
    std::vector<bool>           got_on_any_topic;
    std::vector< boost::any >   any_ptr_list;


    // Methods
    ROS_API();
    // Setup node and start
    bool start(int argc, char **argv, std::string node_name_in=std::string("ROS_interface"));
    // Check if the ROS is started
    bool is_running();

    // Public methods
    std::string get_pkg_path();
    bool update(); // Updating data

private:
    bool _is_initialized;
    bool _set_up_topics();
};


#endif // ROS_API_H
