#ifndef SCENE_W2_H
#define SCENE_W2_H

#include "Scene.h"

//

class SCENE_W2 : public Scene
{
public:
	SCENE_W2(std::string pkg_path_in);

private:
    inline static bool cal_viewport_w(int w, int h, int &cx, int &cy, int &vw, int &vh){
        double asp = 1.5833333333;
        int im_w = w/7;
        int im_h = int(im_w/asp);
        cx = im_w*2;
        cy = 0;
        vw = im_w;
        vh = im_h;
        return true;
    }
};



SCENE_W2::SCENE_W2(std::string pkg_path_in)
{
	_camera_ptr.reset(new ViewManager());
    _camera_ptr->assign_cal_viewport(&cal_viewport_w);

    _pkg_path = (pkg_path_in);
    _Assets_path = (pkg_path_in + "Assets/");


    // Bounding box 2D
    std::shared_ptr<rmBoundingBox2D> _box2D_ptr;


    // Back ground image rmImageDynamicBackground
    // std::shared_ptr<rmImageDynamicBackground> _image_background_2_ptr(new rmImageDynamicBackground(_Assets_path, int(MSG_ID::camera_front_left)) );
    std::shared_ptr<rmImageBoard> _image_background_2_ptr(new rmImageBoard(_Assets_path, int(MSG_ID::camera_front_left), false, false, true) );
    _image_background_2_ptr->alpha = 1.0;
    _image_background_2_ptr->color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_background_2_ptr );

    // Bounding box for front-left camera
    _box2D_ptr.reset(new rmBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_front_all), false, false ) );
    _box2D_ptr->setup_params(608, 384, 608*0, 0);
    // _box2D_ptr->alpha = 0.7;
    _rm_BaseModel.push_back( _box2D_ptr );


}

#endif  // SCENE_W2_H
