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
    std::shared_ptr<rmPointCloud> pc_ptr_1(new rmPointCloud(_Assets_path, int(MSG_ID::point_cloud_1)) );
    pc_ptr_1->set_color(glm::vec3(1.0f));
    _rm_BaseModel.push_back( pc_ptr_1 );
    pc_ptr_1.reset(new rmPointCloud(_Assets_path, int(MSG_ID::point_cloud_map)) );
    pc_ptr_1->set_color(glm::vec3(0.5f, 0.0f, 0.5f));
    _rm_BaseModel.push_back( pc_ptr_1 );

    // Lidar bounding box
    _rm_BaseModel.push_back( std::shared_ptr<rmLidarBoundingBox>(new rmLidarBoundingBox(_Assets_path, int(MSG_ID::lidar_bounding_box_1)) ) );






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

#endif  // SCENE_W1_H
