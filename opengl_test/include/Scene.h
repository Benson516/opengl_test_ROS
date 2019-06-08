#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <string>
// Debug
#include <iostream>

//
#include "ViewManager.h"
#include "rmBaseModel.h"


//

class Scene
{
public:
	Scene(std::string pkg_path_in);
    //
	void MouseEvent(int button,int state,int x,int y);
	void KeyBoardEvent(int key);
	void KeyBoardEvent(unsigned char key);
	void MenuEvent(int item);

	void Render();
	void Update(float dt);

	ViewManager* GetCamera(){ return _camera_ptr; }

private:
    std::string _pkg_path;
    std::string _Assets_path;
	ViewManager* _camera_ptr;

    // Render models
    std::vector<rmBaseModel*> _rm_BaseModel;
};

#endif  // Scene_H
