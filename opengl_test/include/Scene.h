#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <string>
// Debug
#include <iostream>

//
// #include "ViewManager.h"
#include "ViewManager_v2.h"
#include <ROS_ICLU3_v0.hpp>

// Render models
#include "rmBaseModel.h"
#include "rmPointCloud.h"
#include "rmLidarBoundingBox.h"
#include "rmImageBoard.h"
#include "rmBoundingBox2D.h"
#include "rmGrid.h"
// The following are not finished yet
#include "rmSweepingObject.h"
#include "rmText2D.h"
// #include "rmText3D.h"
#include "rmText3D_v2.h"
#include "rmCircle.h"

//

class Scene
{
public:
    Scene();
	Scene(std::string pkg_path_in);
    //
	void Render();
	void Update(float dt);
    void Update(ROS_INTERFACE &ros_interface);
    void Update(ROS_API &ros_api);
    void Reshape(int full_window_width, int full_window_height);

    //
    void MouseEvent(int button,int state,int x,int y);
	void KeyBoardEvent(int key);
	void KeyBoardEvent(unsigned char key, ROS_API &ros_api);
	void MenuEvent(int item);

    // Camera mode
    // Follow, steady, ...etc.
    void switchCameraMode(int mode_in, ROS_API &ros_api);

    // ViewManager
    std::shared_ptr<ViewManager> GetCamera(){ return _camera_ptr; }

protected:
    std::string _pkg_path;
    std::string _Assets_path;
	std::shared_ptr<ViewManager> _camera_ptr;

    int camera_mode;

    // Render models
    std::vector< std::shared_ptr<rmBaseModel> > _rm_BaseModel;
};

#endif  // Scene_H
