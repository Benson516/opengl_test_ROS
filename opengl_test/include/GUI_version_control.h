#ifndef GUI_VERSION_CTRL_H
#define GUI_VERSION_CTRL_H


// The version of ros_api, ros topics and message types
//----------------------------------//
#define __ROS_INTERFACE_VER__   1   // 1, 2
// 1 - FlOWER_EXPOSE
// 2 - HINO
//----------------------------------//

// The version of the class of detection object
//----------------------------------//
#define __DETECTION_OBJ_VER_   1    // 1, 2
// 1 - FlOWER_EXPOSE
// 2 - HINO
//----------------------------------//

// Determin if showing the tracking result or the raw bounding boxes
//----------------------------------//
#define __IS_USING_TRACKING__  1 // 0, 1
// 0 - Without tracking, showing the raw lidar bounding boxes
// 1 - With tracking, showing the tracking boxes
//----------------------------------//


#endif // GUI_VERSION_CTRL_H
