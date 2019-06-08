#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <string>
// Debug
#include <iostream>

//
#include "ViewManager.h"

// Render models
#include "rmBaseModel.h"
#include "rmPointCloud.h"

//

class Scene
{
public:
	Scene(std::string pkg_path_in);
    //
	void Render();
	void Update(float dt);
    std::shared_ptr<ViewManager> GetCamera(){ return _camera_ptr; }
    //
    void MouseEvent(int button,int state,int x,int y);
	void KeyBoardEvent(int key);
	void KeyBoardEvent(unsigned char key);
	void MenuEvent(int item);

private:
    std::string _pkg_path;
    std::string _Assets_path;
	std::shared_ptr<ViewManager> _camera_ptr;

    // Render models
    std::vector< std::shared_ptr<rmBaseModel> > _rm_BaseModel;
    std::vector< std::shared_ptr<rmPointCloud> > _rm_PointCloud;
};

#endif  // Scene_H
