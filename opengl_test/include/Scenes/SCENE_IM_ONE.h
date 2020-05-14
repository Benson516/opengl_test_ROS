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

    int chosed_image_id; // Note: this start from 1, end with 8
    void perSceneKeyBoardEvent(unsigned char key);

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

    bool select_image_with_id(int id);
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

    // The default image showed: center image 11
    chosed_image_id = 5;
    //

    // Image
    std::shared_ptr<rmImageBoard> _image_background_2_ptr;
    // Bounding box 2D
    std::shared_ptr<rmBoundingBox2D> _box2D_ptr;
    // Bounding box 2D (with tag)
    std::shared_ptr<rmlv2TagBoundingBox2D> _box2Dtag_ptr;

    // 00
    // Back ground image rmImageDynamicBackground
    _image_background_2_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_left_fore), false, false, true) );
    _image_background_2_ptr->alpha = 1.0;
    _image_background_2_ptr->color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_background_2_ptr );
    // Bounding box with tag
    _box2Dtag_ptr.reset(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_left_fore), false, false ) );
    _box2Dtag_ptr->setup_params(_IMAGE_W_, _IMAGE_H_, 0, 0);
    _rm_BaseModel.push_back( _box2Dtag_ptr );

    // 01
    // Back ground image rmImageDynamicBackground
    _image_background_2_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_front_top_far), false, false, true) );
    _image_background_2_ptr->alpha = 1.0;
    _image_background_2_ptr->color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_background_2_ptr );
    // Bounding box with tag
    _box2Dtag_ptr.reset(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_front_top_far), false, false ) );
    _box2Dtag_ptr->setup_params(_IMAGE_W_, _IMAGE_H_, 0, 0);
    _rm_BaseModel.push_back( _box2Dtag_ptr );

    // 02
    // Back ground image rmImageDynamicBackground
    _image_background_2_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_right_fore), false, false, true) );
    _image_background_2_ptr->alpha = 1.0;
    _image_background_2_ptr->color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_background_2_ptr );
    // Bounding box with tag
    _box2Dtag_ptr.reset(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_right_fore), false, false ) );
    _box2Dtag_ptr->setup_params(_IMAGE_W_, _IMAGE_H_, 0, 0);
    _rm_BaseModel.push_back( _box2Dtag_ptr );

    // 10
    // Back ground image rmImageDynamicBackground
    _image_background_2_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_left_rear), false, false, true) );
    _image_background_2_ptr->alpha = 1.0;
    _image_background_2_ptr->color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_background_2_ptr );
    // Bounding box with tag
    _box2Dtag_ptr.reset(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_left_rear), false, false ) );
    _box2Dtag_ptr->setup_params(_IMAGE_W_, _IMAGE_H_, 0, 0);
    _rm_BaseModel.push_back( _box2Dtag_ptr );

    // 11
    // Back ground image rmImageDynamicBackground
    _image_background_2_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_front_center), false, false, true) );
    _image_background_2_ptr->alpha = 1.0;
    _image_background_2_ptr->color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_background_2_ptr );
    // Bounding box with tag
    _box2Dtag_ptr.reset(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_front_center), false, false ) );
    _box2Dtag_ptr->setup_params(_IMAGE_W_, _IMAGE_H_, 0, 0);
    _rm_BaseModel.push_back( _box2Dtag_ptr );

    // 12
    // Back ground image rmImageDynamicBackground
    _image_background_2_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_right_rear), false, false, true) );
    _image_background_2_ptr->alpha = 1.0;
    _image_background_2_ptr->color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_background_2_ptr );
    // Bounding box with tag
    _box2Dtag_ptr.reset(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_right_rear), false, false ) );
    _box2Dtag_ptr->setup_params(_IMAGE_W_, _IMAGE_H_, 0, 0);
    _rm_BaseModel.push_back( _box2Dtag_ptr );

    // 21
    // Back ground image rmImageDynamicBackground
    _image_background_2_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_front_top), false, false, true) );
    _image_background_2_ptr->alpha = 1.0;
    _image_background_2_ptr->color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_background_2_ptr );
    // Bounding box with tag
    _box2Dtag_ptr.reset(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_front_top), false, false ) );
    _box2Dtag_ptr->setup_params(_IMAGE_W_, _IMAGE_H_, 0, 0);
    _rm_BaseModel.push_back( _box2Dtag_ptr );

    // 22
    // Back ground image rmImageDynamicBackground
    _image_background_2_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_rear_center), false, false, true) );
    _image_background_2_ptr->alpha = 1.0;
    _image_background_2_ptr->color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_background_2_ptr );
    // Bounding box with tag
    _box2Dtag_ptr.reset(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_rear_center), false, false ) );
    _box2Dtag_ptr->setup_params(_IMAGE_W_, _IMAGE_H_, 0, 0);
    _rm_BaseModel.push_back( _box2Dtag_ptr );


    // Choose image to show
    //-----------------------------------------//
    select_image_with_id( chosed_image_id);
    //-----------------------------------------//

}

bool SCENE_IM_ONE::select_image_with_id(int id){
    /*
                Note: id start from 1
    */
    chosed_image_id = id;
    for (size_t i=0; i < _rm_BaseModel.size(); ++i){
        auto _ptr = &(_rm_BaseModel[i]);
        // (*_ptr)->set_enable( !((*_ptr)->get_enable()) );
        (*_ptr)->set_enable( int(i/2) == (id-1) );
    }
    return true;
}

void SCENE_IM_ONE::perSceneKeyBoardEvent(unsigned char key){

    switch (key)
	{
    case 'i':
    case 'I':
        // Toggle image
        select_image_with_id( (chosed_image_id % 8) + 1 );
        break;
	default:
		break;
	}
    if (key >= '0'){
        unsigned char number = key - '0';
        if (number <=8 && number > 0 ){
            select_image_with_id(int(number));
        }
    }
}



#endif  // SCENE_IM_ONE_H
