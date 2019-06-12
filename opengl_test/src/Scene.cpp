#include "Scene.h"

// ViewManager* Scene::_camera_ptr = new ViewManager();

Scene::Scene(std::string pkg_path_in):
    _pkg_path(pkg_path_in),
    _Assets_path(pkg_path_in + "Assets/")
{
	_camera_ptr.reset(new ViewManager());


    /*
    // Back ground image rmImageStaticBackground
    std::shared_ptr<rmImageStaticBackground> _image_background_ptr(new rmImageStaticBackground(_Assets_path, "view_3.jpg") );
    _image_background_ptr->_alpha = 1.0;
    _image_background_ptr->_color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_background_ptr );
    */

    /*
    // Back ground image rmImageDynamicBackground
    std::shared_ptr<rmImageDynamicBackground> _image_background_2_ptr(new rmImageDynamicBackground(_Assets_path, int(MSG_ID::camera_1)) );
    _image_background_2_ptr->_alpha = 1.0;
    _image_background_2_ptr->_color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_background_2_ptr );
    */

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

    // Lidar bounding box
    _rm_BaseModel.push_back( std::shared_ptr<rmLidarBoundingBox>(new rmLidarBoundingBox(_Assets_path, int(MSG_ID::lidar_bounding_box_1)) ) );

    // static image
    std::shared_ptr<rmImageStatic> image_board_1_ptr(new rmImageStatic(_Assets_path, "clownfish4.png") );
    image_board_1_ptr->Translate(glm::vec3(5.0f, 0.0f, 3.0f));
    image_board_1_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    image_board_1_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    image_board_1_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    image_board_1_ptr->Scale( glm::vec3(3.5f));
    image_board_1_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    _rm_BaseModel.push_back( image_board_1_ptr );



    // std::shared_ptr<rmImageDynamic> dynamic_image_board_1_ptr(new rmImageDynamic(_Assets_path, "view_1.jpg") );
    std::shared_ptr<rmImageDynamic> dynamic_image_board_1_ptr(new rmImageDynamic(_Assets_path, int(MSG_ID::camera_0)) );
    dynamic_image_board_1_ptr->Translate(glm::vec3(0.0f, -10.0f, 3.0f));
    dynamic_image_board_1_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), -M_PI/6.0); // view angle
    dynamic_image_board_1_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    dynamic_image_board_1_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    dynamic_image_board_1_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    dynamic_image_board_1_ptr->Scale( glm::vec3(3.5f));
    dynamic_image_board_1_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    dynamic_image_board_1_ptr->_alpha = 0.7;
    _rm_BaseModel.push_back( dynamic_image_board_1_ptr );

    // std::shared_ptr<rmImageDynamic> dynamic_image_board_1_ptr(new rmImageDynamic(_Assets_path, "view_1.jpg") );
    dynamic_image_board_1_ptr.reset(new rmImageDynamic(_Assets_path, int(MSG_ID::camera_2)) );
    dynamic_image_board_1_ptr->Translate(glm::vec3(0.0f, 10.0f, 3.0f));
    dynamic_image_board_1_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI/6.0); // view angle
    dynamic_image_board_1_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    dynamic_image_board_1_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    dynamic_image_board_1_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    dynamic_image_board_1_ptr->Scale( glm::vec3(3.5f));
    dynamic_image_board_1_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    dynamic_image_board_1_ptr->_alpha = 0.7;
    _rm_BaseModel.push_back( dynamic_image_board_1_ptr );





}

void Scene::Render(){



    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	for (int i = 0; i < _rm_BaseModel.size(); i++){
		_rm_BaseModel[i]->Render(_camera_ptr);
	}
    glDisable(GL_BLEND);
    // glDisable(GL_DEPTH_TEST);
}
void Scene::Update(float dt){
    // rmBaseModel
	for (int i = 0; i < _rm_BaseModel.size(); i++){
		_rm_BaseModel[i]->Update(dt);
	}
}
void Scene::Update(ROS_INTERFACE &ros_interface){
    // Update the "_current_slice_time"
    ros_interface.update_current_slice_time("map", "base");
    ros_interface.set_ref_frame("base");

    /*
    // Camera
    bool is_sucessed = false;
    glm::mat4 _tf_world_by_base = rmBaseModel::ROStf2GLMmatrix( ros_interface.get_tf("base", "map", is_sucessed, false) );
    // glm::mat4 _tf_world_by_base = rmBaseModel::ROStf2GLMmatrix( ros_interface.get_tf("base", "map", is_sucessed, true ) );
    if (is_sucessed){
        std::cout << "Got the camera tf\n";
        _camera_ptr->SetInvCameraModel(_tf_world_by_base);
    }
    */







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

    /*
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
    */
}

void Scene::MenuEvent(int item){

	if (item == 1){
		_rm_BaseModel[1]->Scale(glm::vec3(2.0f, 2.0f, 2.0f));
	}
	else if (item == 2){
		_rm_BaseModel[1]->Scale(glm::vec3(0.5f, 0.5f, 0.5f));
	}
}
