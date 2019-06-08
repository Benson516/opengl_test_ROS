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
    _rm_PointCloud.push_back( std::shared_ptr<rmPointCloud>(new rmPointCloud(_Assets_path) ) );
}

void Scene::Render(){
    // rmBaseModel
	for (int i = 0; i < _rm_BaseModel.size(); i++){
		_rm_BaseModel[i]->Render(_camera_ptr);
	}
    // rmPointCloud
	for (int i = 0; i < _rm_PointCloud.size(); i++){
		_rm_PointCloud[i]->Render(_camera_ptr);
	}
}
void Scene::Update(float dt){
    // rmBaseModel
	for (int i = 0; i < _rm_BaseModel.size(); i++){
		_rm_BaseModel[i]->Update(dt);
	}
    // rmPointCloud
	for (int i = 0; i < _rm_PointCloud.size(); i++){
		_rm_PointCloud[i]->Update(dt);
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
