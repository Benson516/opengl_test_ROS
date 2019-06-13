#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <string>
// Debug
#include <iostream>

//
#include "ViewManager.h"
#include <ROS_ICLU3_v0.hpp>

// Render models
#include "rmBaseModel.h"
#include "rmPointCloud.h"
#include "rmLidarBoundingBox.h"
#include "rmImageStatic.h"
#include "rmImageStaticBackground.h"
#include "rmImageDynamic.h"
#include "rmImageDynamicBackground.h"

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
    std::shared_ptr<ViewManager> GetCamera(){ return _camera_ptr; }
    //
    void MouseEvent(int button,int state,int x,int y);
	void KeyBoardEvent(int key);
	void KeyBoardEvent(unsigned char key);
	void MenuEvent(int item);

protected:
    std::string _pkg_path;
    std::string _Assets_path;
	std::shared_ptr<ViewManager> _camera_ptr;

    // Render models
    std::vector< std::shared_ptr<rmBaseModel> > _rm_BaseModel;
    std::vector< std::shared_ptr<rmPointCloud> > _rm_PointCloud;
};

#endif  // Scene_H
