#ifndef SCENE_IM_ONE_H
#define SCENE_IM_ONE_H

#include "Scene.h"

// Version control
//----------------------------------------//
#include "GUI_version_control.h"
//----------------------------------------//

class SCENE_IM_ONE : public Scene
{
public:
	SCENE_IM_ONE(std::string pkg_path_in);

private:
    inline static bool cal_viewport_w(int w, int h, int &cx, int &cy, int &vw, int &vh){
        double asp = _IMAGE_ASP_;
        int im_w = w; // int(im_h*asp);
        int im_h = h;
        if (im_h*asp > im_w){
            im_h = int(im_w/asp);
        }else{
            im_w = int(im_h*asp);
        }
        cx = 0;
        cy = 0;
        vw = im_w;
        vh = im_h;
        return true;
    }
};



SCENE_IM_ONE::SCENE_IM_ONE(std::string pkg_path_in)
{
	_camera_ptr.reset(new ViewManager());
    // _camera_ptr->assign_cal_viewport(&cal_viewport_w);
    // Layout
    //----------------------------------------//
    attach_cal_viewport_func_ptr(1, &cal_viewport_w);
    switch_layout(0);
    //----------------------------------------//

    _pkg_path = (pkg_path_in);
    _Assets_path = (pkg_path_in + "Assets/");


    // Bounding box 2D
    std::shared_ptr<rmBoundingBox2D> _box2D_ptr;
    // Bounding box 2D (with tag)
    std::shared_ptr<rmlv2TagBoundingBox2D> _box2Dtag_ptr;


    // Back ground image rmImageDynamicBackground
    std::shared_ptr<rmImageBoard> _image_background_2_ptr(new rmImageBoard(_Assets_path, int(MSG_ID::camera_left_fore), false, false, true) );
    _image_background_2_ptr->alpha = 1.0;
    _image_background_2_ptr->color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_background_2_ptr );
    // Bounding box with tag
    _box2Dtag_ptr.reset(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_left_fore), false, false ) );
    _box2Dtag_ptr->setup_params(_IMAGE_W_, _IMAGE_H_, 0, 0);
    _rm_BaseModel.push_back( _box2Dtag_ptr );

}

#endif  // SCENE_IM_ONE_H
