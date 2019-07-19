#ifndef SCENE_W_MAIN_H
#define SCENE_W_MAIN_H

#include "Scene.h"

//

class SCENE_W_main : public Scene
{
public:
	SCENE_W_main(std::string pkg_path_in);

    // Interaction events

    //
    void perSceneROSTopicEvent(ROS_API &ros_api);
    void perSceneKeyBoardEvent(unsigned char key);

    std::vector<size_t> enable_ctr_id_list_image;
    bool is_enable_image3D;
private:
    inline static bool cal_viewport_w(int w, int h, int &cx, int &cy, int &vw, int &vh){
        double asp = 1.5833333333;
        int im_w = w/7;
        int im_h = int(im_w/asp);
        cx = 0;
        cy = im_h;
        vw = w;
        vh = h-im_h;
        return true;
    }
    inline static bool cal_viewport_w_1(int w, int h, int &cx, int &cy, int &vw, int &vh){
        cx = 0;
        cy = 0;
        vw = w;
        vh = h;
        return true;
    }

};


SCENE_W_main::SCENE_W_main(std::string pkg_path_in):
    is_enable_image3D(true)
{
	_camera_ptr.reset(new ViewManager());
    // _camera_ptr->assign_cal_viewport(&cal_viewport_w);
    // Layout
    //----------------------------------------//
    attach_cal_viewport_func_ptr(0, &cal_viewport_w);
    attach_cal_viewport_func_ptr(1, &cal_viewport_w_1);
    switch_layout(0);
    //----------------------------------------//

    _pkg_path = (pkg_path_in);
    _Assets_path = (pkg_path_in + "Assets/");

    // Grid
    std::shared_ptr<rmGrid> _rmGrid_ptr;
    // Image
    std::shared_ptr<rmImageBoard> _image_board_ptr;
    // PointCloud
    std::shared_ptr<rmPointCloud> pc_ptr_1;
    // Text
    std::shared_ptr<rmText2D> _text2D_ptr;
    std::shared_ptr<rmText3D_v2> _text3D_ptr;
    // Bounding box 2D
    std::shared_ptr<rmlv2TagBoundingBox2D> _box2D_ptr;
    // rmlv2PathPlanFake
    std::shared_ptr<rmlv2PathPlanFake> _fake_path_ptr;
    // rmColorBoard
    std::shared_ptr<rmColorBoard> _color_board_ptr;

    /*
    // Back ground image (static)
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, "view_3.jpg", false, false, true) );
    _image_board_ptr->alpha = 1.0;
    _image_board_ptr->color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_board_ptr );
    */

    /*
    // Back ground image (dynamic) front-center camera
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_front_center), false, false, true) );
    _image_board_ptr->alpha = 1.0;
    _image_board_ptr->color_transform = glm::vec4(1.0f);
    _rm_BaseModel.push_back( _image_board_ptr );
    // Bounding box for front-center camera
    _box2D_ptr.reset(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_front_all), false, false ) );
    _box2D_ptr->setup_params(608, 384, 608*1, 0);
    // _box2D_ptr->alpha = 0.7;
    _rm_BaseModel.push_back( _box2D_ptr );
    */








    // Grid ground
    _rmGrid_ptr.reset(new rmGrid(_Assets_path, "map", "base" ) );
    // _rmGrid_ptr->set_grid_param(1.0, 1.0, 10, 10, -6.0f, false);
    _rmGrid_ptr->set_grid_param(1.0, 1.0, 10, 10, -3.0f, true);
    _rm_BaseModel.push_back( _rmGrid_ptr );
    /*
    // Grid local
    _rmGrid_ptr.reset(new rmGrid(_Assets_path, "base", "base" ) );
    _rmGrid_ptr->set_grid_param(1.0, 1.0, 10, 10, -3.0f, false, glm::vec3(0.2f, 0.2f, 0.5f));
    _rm_BaseModel.push_back( _rmGrid_ptr );
    */

    /*
    // rmModelLoaderObj
	std::shared_ptr<rmModelLoaderObj> bottle( new rmModelLoaderObj(_Assets_path, "Potion_bottle.obj", "bottle_mana.png") );
	std::shared_ptr<rmModelLoaderObj> box( new rmModelLoaderObj(_Assets_path, "box_realistic.obj", "box_texture_color.png") );
	bottle->Scale(glm::vec3(0.01, 0.01, 0.01));
	bottle->Rotate(glm::vec3(1, 0, 0), 3.1415926 / 2 * 3);
	bottle->Translate(glm::vec3(0.0, 0.5, 0.0));
	_rm_BaseModel.push_back(bottle);
	_rm_BaseModel.push_back(box);
    */



    // Map
    pc_ptr_1.reset(new rmPointCloud(_Assets_path, int(MSG_ID::point_cloud_map)) );
    pc_ptr_1->set_color(glm::vec3(0.68627451f, 0.0f, 0.76862745f));
    _rm_BaseModel.push_back( pc_ptr_1 );
    // Raw data
    pc_ptr_1.reset(new rmPointCloud(_Assets_path, int(MSG_ID::point_cloud_raw)) );
    pc_ptr_1->set_color(glm::vec3(1.0f));
    _rm_BaseModel.push_back( pc_ptr_1 );



    // rmlv2ObjectTracking
    // _rm_BaseModel.push_back( std::shared_ptr<rmlv2ObjectTracking>(new rmlv2ObjectTracking(_Assets_path, int(MSG_ID::lidar_bounding_box_tracking), "map") ) );

    // Taged Lidar bounding box (tracking, rendering in wire)
    _rm_BaseModel.push_back( std::shared_ptr<rmlv2TagBoundingBox3D>(new rmlv2TagBoundingBox3D(_Assets_path, int(MSG_ID::lidar_bounding_box_tracking)) ) );

    // Lidar bounding box (rendering in face)
    // _rm_BaseModel.push_back( std::shared_ptr<rmLidarBoundingBox>(new rmLidarBoundingBox(_Assets_path, int(MSG_ID::lidar_bounding_box_raw)) ) );


    // Bounding box 2D
    // _rm_BaseModel.push_back( std::shared_ptr<rmlv2TagBoundingBox2D>(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_front_all)) ) );

    // Sweeping object
    // _rm_BaseModel.push_back( std::shared_ptr<rmSweepingObject>(new rmSweepingObject(_Assets_path, "base" ) ) );

    // rmlv2PathPlanFake
    _fake_path_ptr.reset(   new rmlv2PathPlanFake(_Assets_path, int(MSG_ID::vehicle_info) )   );
    _fake_path_ptr->Translate(glm::vec3(-5.5f, 0.0f, 0.0f));
    _rm_BaseModel.push_back( _fake_path_ptr );


    // Circle
    // _rm_BaseModel.push_back( std::shared_ptr<rmCircle>(new rmCircle(_Assets_path, "base" ) ) );
    // rmPolyLines3D
    // _rm_BaseModel.push_back( std::shared_ptr<rmPolyLines3D>(new rmPolyLines3D(_Assets_path, "base") ) );



    /*
    // static image
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, "clownfish4.png", true, true, false) );
    _image_board_ptr->Translate(glm::vec3(5.0f, 0.0f, 3.0f));
    _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    _image_board_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    _image_board_ptr->shape.setBoardSize(11.08, true);
    // _image_board_ptr->Scale( glm::vec3(3.5f));
    // _image_board_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    _rm_BaseModel.push_back( _image_board_ptr );
    */


    // Dynamic image, front-center camera
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_front_center), true, true, false) );
    _image_board_ptr->Translate(glm::vec3(2.77f, 0.0f, 3.0f));
    // _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), 0.0); // view angle
    _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    _image_board_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    _image_board_ptr->shape.setBoardSize(11.08, true);
    // _image_board_ptr->Scale( glm::vec3(3.5f));
    // _image_board_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    _image_board_ptr->alpha = 0.7;
    _rm_BaseModel.push_back( _image_board_ptr );
    // Control list
    enable_ctr_id_list_image.push_back( _rm_BaseModel.size()-1);
    //

    // Bounding box for front-center camera
    _box2D_ptr.reset(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_front_all), true, true ) );
    _box2D_ptr->setup_params(608, 384, 608*1, 0);
    _box2D_ptr->Translate(glm::vec3(2.77f, 0.0f, 3.0f));
    // _box2D_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), 0.0); // view angle
    _box2D_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    _box2D_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    _box2D_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    _box2D_ptr->shape.setBoardSize(11.08, true);
    // _box2D_ptr->Scale( glm::vec3(3.5f));
    // _box2D_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    // _box2D_ptr->alpha = 0.7;
    _rm_BaseModel.push_back( _box2D_ptr );
    // Control list
    enable_ctr_id_list_image.push_back( _rm_BaseModel.size()-1);
    //


    // Dynamic image, front-right camera
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_front_right), true, true, false) );
    _image_board_ptr->Translate(glm::vec3(0.0f, -10.33f, 3.0f));
    _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), -M_PI/6.0); // view angle
    _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    _image_board_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    _image_board_ptr->shape.setBoardSize(11.08, true);
    // _image_board_ptr->Scale( glm::vec3(3.5f));
    // _image_board_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    _image_board_ptr->alpha = 0.7;
    _rm_BaseModel.push_back( _image_board_ptr );
    // Control list
    enable_ctr_id_list_image.push_back( _rm_BaseModel.size()-1);
    //

    // Bounding box for front-right camera
    _box2D_ptr.reset(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_front_all), true, true ) );
    _box2D_ptr->setup_params(608, 384, 608*2, 0);
    _box2D_ptr->Translate(glm::vec3(0.0f, -10.33f, 3.0f));
    _box2D_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), -M_PI/6.0); // view angle
    _box2D_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    _box2D_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    _box2D_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    _box2D_ptr->shape.setBoardSize(11.08, true);
    // _box2D_ptr->Scale( glm::vec3(3.5f));
    // _box2D_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    // _box2D_ptr->alpha = 0.7;
    _rm_BaseModel.push_back( _box2D_ptr );
    // Control list
    enable_ctr_id_list_image.push_back( _rm_BaseModel.size()-1);
    //


    // Dynamic image, front-left camera
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_front_left), true, true, false) );
    _image_board_ptr->Translate(glm::vec3(0.0f, 10.33f, 3.0f));
    _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI/6.0); // view angle
    _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    _image_board_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    _image_board_ptr->shape.setBoardSize(11.08, true);
    // _image_board_ptr->Scale( glm::vec3(3.5f));
    // _image_board_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    _image_board_ptr->alpha = 0.7;
    _rm_BaseModel.push_back( _image_board_ptr );
    // Control list
    enable_ctr_id_list_image.push_back( _rm_BaseModel.size()-1);
    //

    // Bounding box for front-left camera
    _box2D_ptr.reset(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_front_all), true, true ) );
    _box2D_ptr->setup_params(608, 384, 608*0, 0);
    _box2D_ptr->Translate(glm::vec3(0.0f, 10.33f, 3.0f));
    _box2D_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI/6.0); // view angle
    _box2D_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    _box2D_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    _box2D_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    _box2D_ptr->shape.setBoardSize(11.08, true);
    // _box2D_ptr->Scale( glm::vec3(3.5f));
    // _box2D_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    // _box2D_ptr->alpha = 0.7;
    _rm_BaseModel.push_back( _box2D_ptr );
    // Control list
    enable_ctr_id_list_image.push_back( _rm_BaseModel.size()-1);
    //


    /*
    // Dynamic image, front-top camera
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_front_top), true, true, false) );
    _image_board_ptr->Translate(glm::vec3(0.0f, 0.0f, 8.0f));
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), -60.0*M_PI/180.0); // view angle
    _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    _image_board_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    _image_board_ptr->shape.setBoardSize(11.08, true);
    // _image_board_ptr->Scale( glm::vec3(3.5f));
    // _image_board_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    _image_board_ptr->alpha = 0.7;
    _rm_BaseModel.push_back( _image_board_ptr );
    // Control list
    enable_ctr_id_list_image.push_back( _rm_BaseModel.size()-1);
    //

    // Bounding box for front-top camera
    _box2D_ptr.reset(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_front_top), true, true ) );
    _box2D_ptr->setup_params(608, 384, 608*0, 0);
    _box2D_ptr->Translate(glm::vec3(0.0f, 0.0f, 8.0f));
    _box2D_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), -60.0*M_PI/180.0); // view angle
    _box2D_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    _box2D_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    _box2D_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    _box2D_ptr->shape.setBoardSize(11.08, true);
    // _box2D_ptr->Scale( glm::vec3(3.5f));
    // _box2D_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    // _box2D_ptr->alpha = 0.7;
    _rm_BaseModel.push_back( _box2D_ptr );
    // Control list
    enable_ctr_id_list_image.push_back( _rm_BaseModel.size()-1);
    //
    */

    // Dynamic image, front-top camera
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_front_top), true, true, false) );
    _image_board_ptr->Translate(glm::vec3(0.0f, 0.0f, -1.0f));
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), 75.0*M_PI/180.0); // view angle
    _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    _image_board_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    _image_board_ptr->shape.setBoardSize(11.08, true);
    // _image_board_ptr->Scale( glm::vec3(3.5f));
    // _image_board_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    _image_board_ptr->alpha = 0.7;
    _rm_BaseModel.push_back( _image_board_ptr );
    // Control list
    enable_ctr_id_list_image.push_back( _rm_BaseModel.size()-1);
    //

    // Bounding box for front-top camera
    _box2D_ptr.reset(new rmlv2TagBoundingBox2D(_Assets_path, int(MSG_ID::bounding_box_image_front_top), true, true ) );
    _box2D_ptr->setup_params(608, 384, 608*0, 0);
    _box2D_ptr->Translate(glm::vec3(0.0f, 0.0f, -1.0f));
    _box2D_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), 75.0*M_PI/180.0); // view angle
    _box2D_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    _box2D_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    _box2D_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    _box2D_ptr->shape.setBoardSize(11.08, true);
    // _box2D_ptr->Scale( glm::vec3(3.5f));
    // _box2D_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    // _box2D_ptr->alpha = 0.7;
    _rm_BaseModel.push_back( _box2D_ptr );
    // Control list
    enable_ctr_id_list_image.push_back( _rm_BaseModel.size()-1);
    //



    // Dynamic image, rear camera (mirrored)
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_rear_center), true, true, false) );
    _image_board_ptr->Translate(glm::vec3(0.0f, 0.0f, 8.0f));
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), -60.0*M_PI/180.0); // view angle
    // _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    _image_board_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    _image_board_ptr->shape.setBoardSize(11.08, true);
    // _image_board_ptr->Scale( glm::vec3(3.5f));
    // _image_board_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    _image_board_ptr->alpha = 0.7;
    _rm_BaseModel.push_back( _image_board_ptr );
    // Control list
    enable_ctr_id_list_image.push_back( _rm_BaseModel.size()-1);
    //



    // _text3D_ptr.reset( new rmText3D_v2(_Assets_path, "base" ) );
    // _text3D_ptr->Translate(glm::vec3(1.0f, -2.0f, -2.0f));
    // _text3D_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI); // Flip
    // _text3D_ptr->Rotate(glm::vec3(1.0f,0.0f,0.0f), M_PI/2.0);
    // _text3D_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI/2.0);
    // // _text3D_ptr->setBoardSizeRatio(0.5, true);
    // // _text3D_ptr->Translate(glm::vec3(-0.5f, -0.5f, 0.0f));
    // // _text3D_ptr->Scale( glm::vec3(3.5f));
    // // _text3D_ptr->Scale( glm::vec3(4.0f/3.0f, 1.0f, 1.0f));
    // _rm_BaseModel.push_back( _text3D_ptr );


    /*
    _text2D_ptr.reset( new rmText2D() );
    _rm_BaseModel.push_back( _text2D_ptr );
    */


    // test, rmText3D:    400 x "Hello world" --> CPU 104%, GPU 85%
    // test, rmText3D_v2: 400 x "Hello world" --> CPU 60%, GPU 57%
    /*
    for (size_t i=0; i < 400; ++i){
        std::cout << "i = " << i << "\n";
        _text3D_ptr.reset( new rmText3D_v2(_Assets_path ) );
        _text3D_ptr->Translate(glm::vec3(0.0f, 0.0f, (6.0f + 1.0f*i) ));
        _rm_BaseModel.push_back( _text3D_ptr );
    }
    */


    /*
    // rmColorBoard
    _color_board_ptr.reset( new rmColorBoard(_Assets_path, "base", glm::vec4(0.0f, 0.2f, 0.8f, 0.5f), false, true ) );
    _color_board_ptr->shape.setBoardSizePixel(150, 100);
    _color_board_ptr->shape.setBoardPositionCVPixel(-10,10,1,ALIGN_X::RIGHT, ALIGN_Y::TOP );
    _rm_BaseModel.push_back( _color_board_ptr );
    */


    /*
    // Top-level top-centered back image (dynamic) <-- "Rear-sight mirror"
    _image_board_ptr.reset(new rmImageBoard(_Assets_path, int(MSG_ID::camera_rear_center), false, true, true) );
    _image_board_ptr->alpha = 0.9;
    _image_board_ptr->color_transform = glm::vec4(1.0f);
    _image_board_ptr->Rotate(glm::vec3(0.0f,1.0f,0.0f), M_PI); // Flip vertically
    // _image_board_ptr->Rotate(glm::vec3(0.0f,0.0f,1.0f), M_PI/6.0);
    // _image_board_ptr->shape.setBoardSizeRatio(0.2f, false);
    _image_board_ptr->shape.setBoardSizePixel(150, false);
    _image_board_ptr->shape.setBoardPositionCVPixel(-300, 0, 1, ALIGN_X::RIGHT, ALIGN_Y::TOP );
    _rm_BaseModel.push_back( _image_board_ptr );
    */


    // rmlv2SpeedMeter
    _rm_BaseModel.push_back( std::shared_ptr<rmlv2SpeedMeter>( new rmlv2SpeedMeter(_Assets_path, int(MSG_ID::vehicle_info) ) ) );



}



void SCENE_W_main::perSceneKeyBoardEvent(unsigned char key){
    switch (key)
	{
    case 'i':
    case 'I':
        // Toggle enable
        is_enable_image3D = !is_enable_image3D;
        for (size_t i=0; i < enable_ctr_id_list_image.size(); ++i){
            auto _ptr = &(_rm_BaseModel[ enable_ctr_id_list_image[i] ]);
            // (*_ptr)->set_enable( !((*_ptr)->get_enable()) );
            (*_ptr)->set_enable( is_enable_image3D );
        }
        break;
	default:
		break;
	}
}


// Interaction events
//------------------------------------------------------//
void SCENE_W_main::perSceneROSTopicEvent(ROS_API &ros_api){
    std::shared_ptr< opengl_test::GUI2_op > _GUI2_op_ptr;
    bool result = ros_api.get_message( int(MSG_ID::GUI_operatio), _GUI2_op_ptr);
    if (!result){
        return;
    }

    // Filter by "gui_name"
    if (_GUI2_op_ptr->gui_name != ros_api.gui_name){
        return;
    }
    // else
    std::cout << "---\n";
    std::cout << "gui_name: " << _GUI2_op_ptr->gui_name << "\n";
    std::cout << "cam_view_mode: " << _GUI2_op_ptr->cam_view_mode << "\n";
    std::cout << "cam_motion_mode: " << _GUI2_op_ptr->cam_motion_mode << "\n";
    std::cout << "image3D: " << _GUI2_op_ptr->image3D << "\n";
    std::cout << "image_surr: " << _GUI2_op_ptr->image_surr << "\n";
    std::cout << "cam_op: " << _GUI2_op_ptr->cam_op << "\n";
    std::cout << "record_op: " << _GUI2_op_ptr->record_op << "\n";

    // State
    //----------------------------------------//

    // image3D
    if (_GUI2_op_ptr->image3D == "on"){
        // enable
        if (!is_enable_image3D){
            perSceneKeyBoardEvent('i');
        }
    }else if (_GUI2_op_ptr->image3D == "off"){
        // disable
        if (is_enable_image3D){
            perSceneKeyBoardEvent('i');
        }
    }else if (_GUI2_op_ptr->image3D == "toggle"){
        // toggle
        perSceneKeyBoardEvent('i');
    }

    //----------------------------------------//
    // Event
    //----------------------------------------//
    // cam_motion
    if (_GUI2_op_ptr->cam_op == "reset"){
        KeyBoardEvent('z', ros_api);
    }else if (_GUI2_op_ptr->cam_op == "zoom_in"){
        KeyBoardEvent('w', ros_api);
    }else if (_GUI2_op_ptr->cam_op == "zoom_out"){
        KeyBoardEvent('s', ros_api);
    }
    //----------------------------------------//

    //Response
    // Note: only the main window will send back response
    //----------------------------------------//
    opengl_test::GUI2_op res_data;
    res_data.header = _GUI2_op_ptr->header;
    res_data.header.stamp = ros::Time::now(); // Update to current time
    // gui_name
    res_data.gui_name = ros_api.gui_name;
    // cam_motion_mode
    if (camera_view_mode == 0)
        res_data.cam_view_mode = "close";
    else if (camera_view_mode == 1)
        res_data.cam_view_mode = "over";
    else if (camera_view_mode == 2)
        res_data.cam_view_mode = "bird";
    // cam_motion_mode
    if (camera_motion_mode == 0)
        res_data.cam_motion_mode = "follow";
    else if (camera_motion_mode == 1)
        res_data.cam_motion_mode = "static";
    // image3D
    if (is_enable_image3D)
        res_data.image3D = "on";
    else
        res_data.image3D = "off";
    // image_surr
    if (_layout_mode == 0)
        res_data.image_surr = "on";
    else
        res_data.image_surr = "off";
    //----------------------------------------//
    //
    ros_api.ros_interface.send_GUI2_op( int(MSG_ID::GUI_operatio), res_data);
}



#endif  // SCENE_W_MAIN_H