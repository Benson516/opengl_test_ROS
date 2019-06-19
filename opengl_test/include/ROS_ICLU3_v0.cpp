#include <ROS_ICLU3_v0.hpp>


ROS_API::ROS_API():
    _is_initialized(false)
{
    // TODO: replace the following hardcoded path to an auto-detected one
    // path_pkg = "/home/benson516_itri/catkin_ws/src/opengl_test_ROS/opengl_test/";
    path_pkg = ros::package::getPath("opengl_test");
    if (path_pkg.back() != '/'){
        path_pkg += "/";
    }
    std::cout << "path_pkg = <" << path_pkg << ">\n";
}

// Setup node and start
bool ROS_API::start(int argc, char **argv, std::string node_name_in){
    // Setup the ROS interface
    ros_interface.setup_node(argc, argv, node_name_in);
    // Setup topics
    _set_up_topics();
    // start
    return ros_interface.start();
}

// Check if the ROS is started
bool ROS_API::is_running(){
    return ros_interface.is_running();
}

// Get the path of the package
std::string ROS_API::get_pkg_path(){
    return path_pkg;
}

// Updating data
bool ROS_API::update(){
    bool _updated = false;


    // Initialize vectors
    if (!_is_initialized){
        //
        got_on_any_topic.resize( ros_interface.get_count_of_all_topics(), false);
        any_ptr_list.resize( ros_interface.get_count_of_all_topics() );
        msg_time_list.resize( ros_interface.get_count_of_all_topics(), ros::Time(0) );
        //
        _is_initialized = true;
    }
    //

    // All topics
    for (int topic_id=0; topic_id < any_ptr_list.size(); ++topic_id){
        if ( ros_interface.is_topic_a_input(topic_id) ){
            got_on_any_topic[topic_id] = ros_interface.get_any_message(topic_id, any_ptr_list[topic_id], msg_time_list[topic_id] );
            _updated |= got_on_any_topic[topic_id];
        }
    }
    //
    return _updated;
}

// New interfaces - boost::any and (void *)
//---------------------------------------------------------//
bool ROS_API::get_any_message(const int topic_id, boost::any & content_out_ptr){
    if ( !got_on_any_topic[topic_id] ){
        return false;
    }
    content_out_ptr = any_ptr_list[topic_id];
    return true;
}
bool ROS_API::get_any_message(const int topic_id, boost::any & content_out_ptr, ros::Time &msg_stamp){
    if ( !got_on_any_topic[topic_id] ){
        return false;
    }
    content_out_ptr = any_ptr_list[topic_id];
    msg_stamp = msg_time_list[topic_id];
    return true;
}
//---------------------------------------------------------//

//===========================================//
bool ROS_API::_set_up_topics(){
    {
        using MSG::M_TYPE;
        // tfGeoPoseStamped
        ros_interface.add_a_topic("current_pose", int(M_TYPE::tfGeoPoseStamped), true, 10, 1, "map", true, "base");
        // Image
#ifdef __SUB_IMAGES__
        ros_interface.add_a_topic("camera/1/0/image_sync", int(M_TYPE::Image), true, 2, 20);
        ros_interface.add_a_topic("camera/1/1/image_sync", int(M_TYPE::Image), true, 2, 20);
        ros_interface.add_a_topic("camera/1/2/image_sync", int(M_TYPE::Image), true, 2, 20);
        ros_interface.add_a_topic("camera/0/2/image_sync", int(M_TYPE::Image), true, 2, 20);
        ros_interface.add_a_topic("camera/2/0/image", int(M_TYPE::Image), true, 2, 20);
        ros_interface.add_a_topic("camera/2/1/image", int(M_TYPE::Image), true, 2, 20);
        ros_interface.add_a_topic("camera/0/0/image", int(M_TYPE::Image), true, 2, 20);
        ros_interface.add_a_topic("camera/0/1/image", int(M_TYPE::Image), true, 2, 20);
        ros_interface.add_a_topic("camera/2/2/image", int(M_TYPE::Image), true, 2, 20);
#endif // __SUB_IMAGES__
        // ITRIPointCloud
#ifdef __SUB_POINT_CLOUD__
        ros_interface.add_a_topic("LidFrontLeft_sync", int(M_TYPE::ITRIPointCloud), true, 2, 20, "base");
        ros_interface.add_a_topic("points_map", int(M_TYPE::PointCloud2), true, 2, 20, "map");
#endif // __SUB_POINT_CLOUD__
        ros_interface.add_a_topic("LidRoi", int(M_TYPE::ITRI3DBoundingBox), true, 10, 20, "base");
        ros_interface.add_a_topic("CamMsg", int(M_TYPE::ITRICamObj), true, 10, 20, "base");

        /*
        // Counts
        num_Image = ros_interface.get_count_of_a_topic_type(M_TYPE::Image);
        num_ITRIPointCloud = ros_interface.get_count_of_a_topic_type(M_TYPE::ITRIPointCloud);
        num_ITRIPointCloud += ros_interface.get_count_of_a_topic_type(M_TYPE::PointCloud2);
        */
    }
    //------------------------------------------------//
}
