#include "Scene.h"

// Default constructor, derived class will call this
Scene::Scene():
    camera_mode(0)
{

}
Scene::Scene(std::string pkg_path_in):
    camera_mode(0)
{
	_camera_ptr.reset(new ViewManager());
    _pkg_path = (pkg_path_in);
    _Assets_path = (pkg_path_in + "Assets/");

    // Image
    std::shared_ptr<rmImageBoard> _image_board_ptr;
    // PointCloud
    std::shared_ptr<rmPointCloud> pc_ptr_1;

    /*
    // Back ground image (static)
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, "view_3.jpg", false, false, true) );
    _image_board_ptr->alpha = 1.0;
    _image_board_ptr->color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_board_ptr );
    */

    /*
    // Back ground image (dynamic) front camera
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_1), false, false, true) );
    _image_board_ptr->alpha = 1.0;
    _image_board_ptr->color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_board_ptr );
    */

    /*
    // Top-level top-centered image (dynamic) <-- "Rear-sight mirror"
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_8), false, true, true) );
    _image_board_ptr->alpha = 1.0;
    _image_board_ptr->color_transform = glm::vec4(1.0f);
    // _image_board_ptr->Translate(glm::vec3(0.0f, 0.0f, 1.0f)); // Move to ackground
    _image_board_ptr->Translate(glm::vec3(0.0f, 0.8f, 0.0f)); // Move to up-center
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI); // Flip vertically
    // _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI/6.0);
    _image_board_ptr->Scale( glm::vec3(0.2f, 0.2f, 0.5f));
    _rm_BaseModel.push_back( _image_board_ptr );
    */


    // BaseModel
	std::shared_ptr<rmBaseModel> bottle( new rmBaseModel(_Assets_path, "Potion_bottle.obj", "bottle_mana.png") );
	std::shared_ptr<rmBaseModel> box( new rmBaseModel(_Assets_path, "box_realistic.obj", "box_texture_color.png") );
	bottle->Scale(glm::vec3(0.01, 0.01, 0.01));
	bottle->Rotate(glm::vec3(1, 0, 0), 3.1415926 / 2 * 3);
	bottle->Translate(glm::vec3(0.0, 0.5, 0.0));
	_rm_BaseModel.push_back(bottle);
	_rm_BaseModel.push_back(box);




    // Map
    pc_ptr_1.reset(new rmPointCloud(_Assets_path, int(MSG_ID::point_cloud_map)) );
    pc_ptr_1->set_color(glm::vec3(0.5f, 0.0f, 0.5f));
    _rm_BaseModel.push_back( pc_ptr_1 );
    // Raw data
    pc_ptr_1.reset(new rmPointCloud(_Assets_path, int(MSG_ID::point_cloud_1)) );
    pc_ptr_1->set_color(glm::vec3(1.0f));
    _rm_BaseModel.push_back( pc_ptr_1 );



    // Lidar bounding box
    _rm_BaseModel.push_back( std::shared_ptr<rmLidarBoundingBox>(new rmLidarBoundingBox(_Assets_path, int(MSG_ID::lidar_bounding_box_1)) ) );




    // static image
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, "clownfish4.png", true, true, false) );
    _image_board_ptr->Translate(glm::vec3(5.0f, 0.0f, 3.0f));
    _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    _image_board_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    _image_board_ptr->Scale( glm::vec3(3.5f));
    _image_board_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    _rm_BaseModel.push_back( _image_board_ptr );


    // Dynamic image, front-right camera
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_0), true, true, false) );
    _image_board_ptr->Translate(glm::vec3(0.0f, -10.0f, 3.0f));
    _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), -M_PI/6.0); // view angle
    _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    _image_board_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    _image_board_ptr->Scale( glm::vec3(3.5f));
    _image_board_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    _image_board_ptr->alpha = 0.7;
    _rm_BaseModel.push_back( _image_board_ptr );

    // Dynamic image, front-left camera
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_2), true, true, false) );
    _image_board_ptr->Translate(glm::vec3(0.0f, 10.0f, 3.0f));
    _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI/6.0); // view angle
    _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    _image_board_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    _image_board_ptr->Scale( glm::vec3(3.5f));
    _image_board_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    _image_board_ptr->alpha = 0.7;
    _rm_BaseModel.push_back( _image_board_ptr );

    /*
    // Dynamic image, front-down camera
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_3), true, true, false) );
    _image_board_ptr->Translate(glm::vec3(-3.0f, 0.0f, 3.0f));
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), -M_PI/6.0); // view angle
    _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    _image_board_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    _image_board_ptr->Scale( glm::vec3(3.5f));
    _image_board_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    _image_board_ptr->alpha = 0.7;
    _rm_BaseModel.push_back( _image_board_ptr );
    */




}

void Scene::Render(){

    // test, set viewport and reset screen
    _camera_ptr->SwitchGLViewPortAndCleanDraw();
    //


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
    // Update the "_latest_tf_common_update_time"
    // ros_interface.update_latest_tf_common_update_time("map", "base");
    // ros_interface.set_global_delay(0.3);
    // ros_interface.update_current_slice_time();
    // ros_interface.set_ref_frame("base");

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

    // evaluation
    // TIME_STAMP::Period period_Update("Update");
    // rmBaseModel
	for (int i = 0; i < _rm_BaseModel.size(); i++){
		_rm_BaseModel[i]->Update(ros_interface);
        // evaluation
        // period_Update.stamp(); period_Update.show_usec();
        //
	}
}
void Scene::Update(ROS_API &ros_api){
    // Update the "_latest_tf_common_update_time"
    // ros_interface.update_latest_tf_common_update_time("map", "base");
    // ros_interface.set_global_delay(0.3);
    // ros_interface.update_current_slice_time();


    switch(camera_mode){
        case 0: // Follow
            ros_api.ros_interface.set_ref_frame("base");
            break;
        case 1: // Steady
            ros_api.ros_interface.set_ref_frame("map");
            break;
        default:
            break;
    }


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

    // evaluation
    // TIME_STAMP::Period period_Update("Update");
    // rmBaseModel
	for (int i = 0; i < _rm_BaseModel.size(); i++){
		_rm_BaseModel[i]->Update(ros_api);
        // evaluation
        // period_Update.stamp(); period_Update.show_usec();
        //
	}
}

void Scene::Reshape(int full_window_width, int full_window_height){

}

void Scene::MouseEvent(int button, int state, int x, int y){
	_camera_ptr->mouseEvents(button, state, x, y);
}

void Scene::KeyBoardEvent(int key){

}

void Scene::KeyBoardEvent(unsigned char key, ROS_API &ros_api){



	switch (key)
	{
    /*
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
    */
    case 'z':
	case 'Z':
        if (camera_mode == 1){
            // Camera reference pose
            bool is_sucessed = false;
            glm::mat4 _tf_world_by_base = rmBaseModel::ROStf2GLMmatrix( ros_api.ros_interface.get_tf("base", "map", is_sucessed, false) );
            // glm::mat4 _tf_world_by_base = rmBaseModel::ROStf2GLMmatrix( ros_interface.get_tf("base", "map", is_sucessed, true ) );
            if (is_sucessed){
                _camera_ptr->SetDefaultCameraModelInv( _tf_world_by_base );
            }
        }
		break;
    case 'c':
	case 'C':
		switchCameraMode( (camera_mode+1)%2 , ros_api);
		break;
	default:
		break;
	}

    // Camera operations
    _camera_ptr->keyEvents(key);

}

void Scene::MenuEvent(int item){

	if (item == 1){
		_rm_BaseModel[1]->Scale(glm::vec3(2.0f, 2.0f, 2.0f));
	}
	else if (item == 2){
		_rm_BaseModel[1]->Scale(glm::vec3(0.5f, 0.5f, 0.5f));
	}
}


void Scene::switchCameraMode(int mode_in, ROS_API &ros_api){
    camera_mode = mode_in;
    //
    switch(camera_mode){
        case 0: // Follow
            {
                // Set the default view matrix
                glm::vec3 eyePosition(0.0f, 0.0f, 12.0f);
            	glm::vec3 eyeLookPosition(0.0f, 0.0f, 0.0f);
            	glm::vec3 up(0, 1, 0);
                _camera_ptr->SetDefaultViewMatrix( lookAt(eyePosition, eyeLookPosition, up) );
                // Set the default camera model matrix (the inverse)
            	glm::mat4 translationMatrix(1.0);
            	glm::mat4 rotationMatrix(1.0);
                rotationMatrix = glm::rotate(rotationMatrix, deg2rad(90.0f), glm::vec3(0.0f, 0.0f, 1.0f)); // z-axis
                rotationMatrix = glm::rotate(rotationMatrix, deg2rad(75.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // y-axis
                _camera_ptr->SetDefaultTansformMatrix( translationMatrix*rotationMatrix );
                _camera_ptr->SetDefaultCameraModelInv( glm::mat4(1.0) );
                _camera_ptr->Reset();
                break;
            }
        case 1: // Steady
            {
                // Set the default view matrix
                glm::vec3 eyePosition(0.0f, 0.0f, 12.0f);
            	glm::vec3 eyeLookPosition(0.0f, 0.0f, 0.0f);
            	glm::vec3 up(0, 1, 0);
                _camera_ptr->SetDefaultViewMatrix( lookAt(eyePosition, eyeLookPosition, up) );
                // Set the default camera model matrix (the inverse)
                glm::mat4 translationMatrix(1.0);
                glm::mat4 rotationMatrix(1.0);
                rotationMatrix = glm::rotate(rotationMatrix, deg2rad(90.0f), glm::vec3(0.0f, 0.0f, 1.0f)); // z-axis
                // rotationMatrix = glm::rotate(rotationMatrix, deg2rad(75.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // y-axis
                rotationMatrix = glm::rotate(rotationMatrix, deg2rad(45.0f), glm::vec3(0.0f, 1.0f, 0.0f)); // y-axis
                _camera_ptr->SetDefaultTansformMatrix( translationMatrix*rotationMatrix );
                // Camera reference pose
                bool is_sucessed = false;
                glm::mat4 _tf_world_by_base = rmBaseModel::ROStf2GLMmatrix( ros_api.ros_interface.get_tf("base", "map", is_sucessed, false) );
                // glm::mat4 _tf_world_by_base = rmBaseModel::ROStf2GLMmatrix( ros_interface.get_tf("base", "map", is_sucessed, true ) );
                if (is_sucessed){
                    _camera_ptr->SetDefaultCameraModelInv( _tf_world_by_base );
                }
                _camera_ptr->Reset();
                break;
            }
        default:
            break;
    }
}
