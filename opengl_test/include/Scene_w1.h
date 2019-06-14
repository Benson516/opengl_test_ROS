#ifndef SCENE_W1_H
#define SCENE_W1_H

#include "Scene.h"

//

class SCENE_W1 : public Scene
{
public:
	SCENE_W1(std::string pkg_path_in);

private:

};

SCENE_W1::SCENE_W1(std::string pkg_path_in)
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
    _image_board_ptr->_alpha = 1.0;
    _image_board_ptr->_color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_board_ptr );
    */

    /*
    // Back ground image (dynamic) front camera
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_1), false, false, true) );
    _image_board_ptr->_alpha = 1.0;
    _image_board_ptr->_color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_board_ptr );
    */


    /*
    // Top-level top-centered image (dynamic) <-- "Rear-sight mirror"
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_8), false, true, true) );
    _image_board_ptr->_alpha = 1.0;
    _image_board_ptr->_color_transform = glm::vec4(1.0f);
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
    pc_ptr_1->set_color(glm::vec3(0.68627451f, 0.0f, 0.76862745f));
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
    _image_board_ptr->_alpha = 0.7;
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
    _image_board_ptr->_alpha = 0.7;
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
    _image_board_ptr->_alpha = 0.7;
    _rm_BaseModel.push_back( _image_board_ptr );
    */




}

#endif  // SCENE_W1_H
