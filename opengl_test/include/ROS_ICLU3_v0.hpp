#ifndef ROS_API_H
#define ROS_API_H

// #include <ROS_interface.hpp>
// #include <ROS_interface_v2.hpp>
// #include <ROS_interface_v3.hpp>
#include <ROS_interface_v4.hpp>

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
    bounding_box_image_front_1,
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
    std::vector< ros::Time >    msg_time_list;


    // Methods
    ROS_API();
    // Setup node and start
    bool start(int argc, char **argv, std::string node_name_in=std::string("ROS_interface"));
    // Check if the ROS is started
    bool is_running();

    // Public methods
    std::string get_pkg_path();
    bool update(); // Updating data

    // New interfaces - boost::any and (void *)
    //---------------------------------------------------------//
    bool get_any_message(const int topic_id, boost::any & content_out_ptr);
    bool get_any_message(const int topic_id, boost::any & content_out_ptr, ros::Time &msg_stamp);
    //---------------------------------------------------------//

private:
    bool _is_initialized;
    bool _set_up_topics();
};



/*
// Using put_any() with content_in: (The following syntex makes a clear poiter transfer withoud copying or sharring)
//---------------------------------------//
boost::any any_ptr;
{
    std::shared_ptr< _T > _content_ptr = std::make_shared< _T >( content_in ); // <-- If we already get the std::shared_ptr, ignore this line
    any_ptr = _content_ptr;
} // <-- Note: the _content_ptr is destroyed when leaveing the scope, thus the use_count for the _ptr in any_ptr is "1" (unique).
buffwr_obj.put_any(any_ptr, true, _time_in, true);
//---------------------------------------//

// Using put_any() with content_in_ptr: (The following syntex makes a clear poiter transfer withoud copying or sharring)
//---------------------------------------//
boost::any any_ptr;
{
    std::shared_ptr< _T > _content_ptr = std::make_shared< _T >( *content_in_ptr ); // <-- If we already get the std::shared_ptr, ignore this line
    any_ptr = _content_ptr;
} // <-- Note: the _content_ptr is destroyed when leaveing the scope, thus the use_count for the _ptr in any_ptr is "1" (unique).
buffwr_obj.put_any(any_ptr, true, _time_in, true);
//---------------------------------------//

// Using front_any() with content_out_ptr: (The following syntex makes a clear poiter transfer withoud copying or sharring)
//---------------------------------------//
std::shared_ptr< _T > content_out_ptr;
{
    boost::any any_ptr;
    bool result = buffwr_list[topic_id]->front_any(any_ptr, true, _current_slice_time);
    if (result){
        // content_out_ptr = boost::any_cast< std::shared_ptr< cv::Mat > >( any_ptr ); // <-- Not good, this makes a copy
        std::shared_ptr< cv::Mat > *_ptr_ptr = boost::any_cast< std::shared_ptr< cv::Mat > >( &any_ptr );
        content_out_ptr = *_ptr_ptr;
    }
} // <-- Note: the any_ptr is destroyed when leaving this scope, thus the use_count for content_out_ptr is "1" (unique).
//---------------------------------------//
*/

#endif // ROS_API_H
