#ifndef SCENE_W2_H
#define SCENE_W2_H

#include "Scene.h"

//

class SCENE_W2 : public Scene
{
public:
	SCENE_W2(std::string pkg_path_in);

private:

};

bool cal_viewport_w2(int w, int h, int &cx, int &cy, int &vw, int &vh){
    double asp = 1.5833333333;
    int im_w = w/7;
    int im_h = int(im_w/asp);
    cx = im_w*2;
    cy = 0;
    vw = im_w;
    vh = im_h;
    return true;
}

SCENE_W2::SCENE_W2(std::string pkg_path_in)
{
	_camera_ptr.reset(new ViewManager());
    _camera_ptr->assign_cal_viewport(&cal_viewport_w2);

    _pkg_path = (pkg_path_in);
    _Assets_path = (pkg_path_in + "Assets/");





    // Back ground image rmImageDynamicBackground
    // std::shared_ptr<rmImageDynamicBackground> _image_background_2_ptr(new rmImageDynamicBackground(_Assets_path, int(MSG_ID::camera_2)) );
    std::shared_ptr<rmImageBoard> _image_background_2_ptr(new rmImageBoard(_Assets_path, int(MSG_ID::camera_2), false, false, true) );
    _image_background_2_ptr->_alpha = 1.0;
    _image_background_2_ptr->_color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_background_2_ptr );



}

#endif  // SCENE_W2_H
