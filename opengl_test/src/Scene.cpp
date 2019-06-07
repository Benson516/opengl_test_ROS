#include "Scene.h"

// ViewManager* Scene::_camera_ptr = new ViewManager();

Scene::Scene(std::string pkg_path_in):
    _pkg_path(pkg_path_in),
    _Assets_path(pkg_path_in + "Assets/")
{
	_camera_ptr = new ViewManager();

	BaseModel* bottle = new BaseModel(_Assets_path, "Potion_bottle.obj", "bottle_mana.png");
	BaseModel* box = new BaseModel(_Assets_path, "box_realistic.obj", "box_texture_color.png");

	bottle->Scale(glm::vec3(0.01, 0.01, 0.01));
	bottle->Rotate(glm::vec3(1, 0, 0), 3.1415926 / 2 * 3);
	bottle->Translate(glm::vec3(0.0, 0.5, 0.0));

	models.push_back(bottle);
	models.push_back(box);
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
		models[1]->Rotate(glm::vec3(0,1,0),-0.1f);
		break;
	case 'x':
	case 'X':
		models[1]->Rotate(glm::vec3(0, 1, 0), 0.1f);
		break;
	case 'c':
	case 'C':
		models[1]->Translate(glm::vec3(-0.1, 0, 0));
		break;
	case 'v':
	case 'V':
		models[1]->Translate(glm::vec3(0.1, 0, 0));
		break;
	default:
		break;
	}
}

void Scene::MenuEvent(int item){

	if (item == 1){
		models[1]->Scale(glm::vec3(2.0f, 2.0f, 2.0f));
	}
	else if (item == 2){
		models[1]->Scale(glm::vec3(0.5f, 0.5f, 0.5f));
	}
}

void Scene::Render(){
	for (int i = 0; i < models.size(); i++){
		models[i]->Render(_camera_ptr);
	}
}
void Scene::Update(float dt){
	for (int i = 0; i < models.size(); i++){
		models[i]->Update(dt);
	}
}

std::vector<BaseModel*> Scene::GetModels(){
	return models;
}
