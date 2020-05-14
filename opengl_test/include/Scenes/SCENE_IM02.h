#ifndef SCENE_IM02_H
#define SCENE_IM02_H

#include "Scene.h"

// Version control
//----------------------------------------//
#include "GUI_version_control.h"
//----------------------------------------//

class SCENE_IM02 : public Scene
{
public:
	SCENE_IM02(std::string pkg_path_in);

private:
    inline static bool cal_viewport_w(int w, int h, int &cx, int &cy, int &vw, int &vh){
        double asp = _IMAGE_ASP_;
        int im_h = h/3;
        int im_w = int(im_h*asp);
        cx = im_w*2;
        cy = im_h*(2-0);
        vw = im_w;
        vh = im_h;
        return true;
    }
};



SCENE_IM02::SCENE_IM02(std::string pkg_path_in)
{
	_camera_ptr.reset(new ViewManager());
    // _camera_ptr->assign_cal_viewport(&cal_viewport_w);
    // Layout
    //----------------------------------------//
    attach_cal_viewport_func_ptr(0, &cal_viewport_w);
    switch_layout(0);
    //----------------------------------------//

    _pkg_path = (pkg_path_in);
    _Assets_path = (pkg_path_in + "Assets/");


    // Bounding box 2D
    std::shared_ptr<rmBoundingBox2D> _box2D_ptr;
    // Bounding box 2D (with tag)
    std::shared_ptr<rmlv2TagBoundingBox2D> _box2Dtag_ptr;


    // Back ground image rmImageDynamicBackground
    std::shared_ptr<rmImageBoard> _image_background_2_ptr(new rmImageBoard(_Assets_path, int(MSG_ID::camera_right_fore), false, false, true) );
    _image_background_2_ptr->alpha = 1.0;
    _image_background_2_ptr->color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_background_2_ptr );
    // Bounding box with tag
    _box2Dtag_ptr.reset(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_right_fore), false, false ) );
    _box2Dtag_ptr->setup_params(_IMAGE_W_, _IMAGE_H_, 0, 0);
    _rm_BaseModel.push_back( _box2Dtag_ptr );

}

#endif  // SCENE_IM02_H
