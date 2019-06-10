#include <ROS_ICLU3_v0.hpp>


ROS_ICLU3_V0::ROS_ICLU3_V0():
    _is_initialized(false),
    num_Image(0), num_ITRIPointCloud(0)
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
bool ROS_ICLU3_V0::start(int argc, char **argv, std::string node_name_in){
    // Setup the ROS interface
    ros_interface.setup_node(argc, argv, node_name_in);
    // Setup topics
    _set_up_topics();
    // start
    return ros_interface.start();
}

// Check if the ROS is started
bool ROS_ICLU3_V0::is_running(){
    return ros_interface.is_running();
}

// Get the path of the package
std::string ROS_ICLU3_V0::get_pkg_path(){
    return path_pkg;
}

// Updating data
bool ROS_ICLU3_V0::update(){
    bool _updated = false;
    int _type_head_id = 0;
    // Note: the topics of same types should be added adjacent to each other
    //       so that the following codes work.

    // Initialize vectors
    if (!_is_initialized){
        // Image
        Image_ptr_list.resize(num_Image);
        got_Image.resize(num_Image);
        // ITRIPointCloud
        ITRIPointCloud_ptr_list.resize(num_ITRIPointCloud);
        got_ITRIPointCloud.resize(num_ITRIPointCloud);
        //
        _is_initialized = true;
    }
    //

    // Image
    _type_head_id = int(MSG_ID::camera_0);
    for (size_t i=0; i < num_Image; ++i){
        got_Image[i] = ros_interface.get_Image( (_type_head_id+i), Image_ptr_list[i]);
        _updated |= got_Image[i];
    }
    // ITRIPointCloud
    _type_head_id = int(MSG_ID::point_cloud_1);
    for (size_t i=0; i < num_ITRIPointCloud; ++i){
        got_ITRIPointCloud[i] = ros_interface.get_ITRIPointCloud( (_type_head_id+i), ITRIPointCloud_ptr_list[i]);
        _updated |= got_ITRIPointCloud[i];
    }
    //
    return _updated;
}


//===========================================//
bool ROS_ICLU3_V0::_set_up_topics(){
    {
        using MSG::M_TYPE;
        // Image
#ifdef __SUB_IMAGES__
        ros_interface.add_a_topic("/camera/1/0/image_sync", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/1/1/image_sync", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/1/2/image_sync", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/0/2/image_sync", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/2/0/image", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/2/1/image", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/0/0/image", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/0/1/image", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/2/2/image", int(M_TYPE::Image), true, 1, 3);
        num_Image = 9; // Hand coded for now..
#endif // __SUB_IMAGES__
        // ITRIPointCloud
#ifdef __SUB_POINT_CLOUD__
        ros_interface.add_a_topic("LidFrontLeft_sync", int(M_TYPE::ITRIPointCloud), true, 5, 5);
        num_ITRIPointCloud = 1; // Hand coded for now..
#endif // __SUB_POINT_CLOUD__
    }
    //------------------------------------------------//
}
