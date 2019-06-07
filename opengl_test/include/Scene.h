#ifndef SCENE_H
#define SCENE_H

#include <vector>
#include <string>
// Debug
#include <iostream>

//
#include "ViewManager.h"
#include "BaseModel.h"


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
	std::vector<BaseModel*> GetModels();

private:
    std::string _pkg_path;
    std::string _Assets_path;
	std::vector<BaseModel*> models;
	ViewManager* _camera_ptr;
};

#endif  // Scene_H
