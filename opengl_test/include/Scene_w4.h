#ifndef SCENE_W4_H
#define SCENE_W4_H

#include "Scene.h"

//

class SCENE_W4 : public Scene
{
public:
	SCENE_W4(std::string pkg_path_in);

private:

};

SCENE_W4::SCENE_W4(std::string pkg_path_in)
{
	_camera_ptr.reset(new ViewManager());
    _pkg_path = (pkg_path_in);
    _Assets_path = (pkg_path_in + "Assets/");





    // Back ground image rmImageDynamicBackground
    std::shared_ptr<rmImageDynamicBackground> _image_background_2_ptr(new rmImageDynamicBackground(_Assets_path, int(MSG_ID::camera_0)) );
    _image_background_2_ptr->_alpha = 1.0;
    _image_background_2_ptr->_color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_background_2_ptr );



}

#endif  // SCENE_W4_H
