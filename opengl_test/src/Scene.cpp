#include "Scene.h"

// ViewManager* Scene::_camera_ptr = new ViewManager();

Scene::Scene(std::string pkg_path_in):
    _pkg_path(pkg_path_in),
    _Assets_path(pkg_path_in + "Assets/")
{
	_camera_ptr.reset(new ViewManager());

    // BaseModel
	std::shared_ptr<rmBaseModel> bottle( new rmBaseModel(_Assets_path, "Potion_bottle.obj", "bottle_mana.png") );
	std::shared_ptr<rmBaseModel> box( new rmBaseModel(_Assets_path, "box_realistic.obj", "box_texture_color.png") );
	bottle->Scale(glm::vec3(0.01, 0.01, 0.01));
	bottle->Rotate(glm::vec3(1, 0, 0), 3.1415926 / 2 * 3);
	bottle->Translate(glm::vec3(0.0, 0.5, 0.0));
	_rm_BaseModel.push_back(bottle);
	_rm_BaseModel.push_back(box);

    // PointCloud
    _rm_BaseModel.push_back( std::shared_ptr<rmPointCloud>(new rmPointCloud(_Assets_path, int(MSG_ID::point_cloud_1)) ) );
}

void Scene::Render(){
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	for (int i = 0; i < _rm_BaseModel.size(); i++){
		_rm_BaseModel[i]->Render(_camera_ptr);
	}
    glDisable(GL_BLEND);
}
void Scene::Update(float dt){
    // rmBaseModel
	for (int i = 0; i < _rm_BaseModel.size(); i++){
		_rm_BaseModel[i]->Update(dt);
	}
}
void Scene::Update(ROS_INTERFACE &ros_interface){
    // rmBaseModel
	for (int i = 0; i < _rm_BaseModel.size(); i++){
		_rm_BaseModel[i]->Update(ros_interface);
	}
}


void Scene::MouseEvent(int button, int state, int x, int y){
	_camera_ptr->mouseEvents(button, state, x, y);
}

void Scene::KeyBoardEvent(int key){

}

void Scene::KeyBoardEvent(unsigned char key){
	_camera_ptr->keyEvents(key);

	switch (key)
	{
	case 'z':
	case 'Z':
		_rm_BaseModel[1]->Rotate(glm::vec3(0,1,0),-0.1f);
		break;
	case 'x':
	case 'X':
		_rm_BaseModel[1]->Rotate(glm::vec3(0, 1, 0), 0.1f);
		break;
	case 'c':
	case 'C':
		_rm_BaseModel[1]->Translate(glm::vec3(-0.1, 0, 0));
		break;
	case 'v':
	case 'V':
		_rm_BaseModel[1]->Translate(glm::vec3(0.1, 0, 0));
		break;
	default:
		break;
	}
}

void Scene::MenuEvent(int item){

	if (item == 1){
		_rm_BaseModel[1]->Scale(glm::vec3(2.0f, 2.0f, 2.0f));
	}
	else if (item == 2){
		_rm_BaseModel[1]->Scale(glm::vec3(0.5f, 0.5f, 0.5f));
	}
}
