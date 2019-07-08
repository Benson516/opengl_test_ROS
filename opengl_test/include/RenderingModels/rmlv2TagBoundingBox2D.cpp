#include "rmlv2TagBoundingBox2D.h"


namespace rmlv2TagBoundingBox2D_ns{
    // Predefined colors
    //-------------------------------------------//
    #define NUM_OBJ_CLASS 8
    #define color_normalize_factor  (1.0f/255.0f)
    glm::vec3 default_class_color(50, 50, 50);
    glm::vec3 obj_class_colors[] = {
        glm::vec3(50, 50, 255), // person
        glm::vec3(255, 153, 102), // bicycle
        glm::vec3(153, 255, 255), // car
        glm::vec3(255, 153, 127), // motorbike
        glm::vec3(255, 255, 0), // not showing aeroplane
        glm::vec3(102, 204, 255), // bus
        glm::vec3(255, 255, 100), // not showing train
        glm::vec3(255, 153, 102), // truck
        glm::vec3(50, 50, 50) // default
    };
    glm::vec3 get_obj_class_color(int obj_class_in){
        if (obj_class_in < NUM_OBJ_CLASS){
            return ( obj_class_colors[obj_class_in] * color_normalize_factor );
        }
        return ( default_class_color * color_normalize_factor );
    }
    //-------------------------------------------//

}

rmlv2TagBoundingBox2D::rmlv2TagBoundingBox2D(
    std::string _path_Assets_in,
    int _ROS_topic_id_in,
    bool is_perspected_in,
    bool is_moveable_in
):
    is_perspected(is_perspected_in),
    is_moveable(is_moveable_in),
    _ROS_topic_id(_ROS_topic_id_in),
    //
    rm_box(_path_Assets_in, _ROS_topic_id_in, is_perspected_in, is_moveable_in),
    rm_text(_path_Assets_in, _ROS_topic_id_in)
{
    //
	Init();
}
void rmlv2TagBoundingBox2D::Init(){


    // For adjusting the model pose by public methods
    attach_pose_model_by_model_ref_ptr( *rm_box.get_model_m_ptr() );
    attach_pose_model_by_model_ref_ptr( *rm_text.get_model_m_ptr() );

}

void rmlv2TagBoundingBox2D::Update(float dt){
    // Update the data (buffer variables) here
}
void rmlv2TagBoundingBox2D::Update(ROS_INTERFACE &ros_interface){
    // Update the data (buffer variables) here
}

void rmlv2TagBoundingBox2D::Update(ROS_API &ros_api){
    // Update the data (buffer variables) here
    // test, use transform
    ros::Time msg_time;
    bool _result = false;
    _result = ros_api.get_message(_ROS_topic_id, msg_out_ptr, msg_time);

    if (_result){
        update_GL_data();
        // rm_text.insert_text();
    }

    //
    rm_box.Update(ros_api);
    rm_text.Update(ros_api);
}


void rmlv2TagBoundingBox2D::Render(std::shared_ptr<ViewManager> &_camera_ptr){

    rm_box.Render(_camera_ptr);
    rm_text.Render(_camera_ptr);
}


/*
void rmlv2TagBoundingBox2D::update_GL_data(){
    // Reset
    if (is_perspected){
        text2Din3D_list.clear();
    }else{
        text2Dflat_list.clear();
    }

    //
    if (msg_out_ptr->camObj.size() == 0){
        // Insert texts
        if (is_perspected){
            rm_text.insert_text(text2Din3D_list);
        }else{
            rm_text.insert_text(text2Dflat_list);
        }
        return;
    }
    long long num_box = msg_out_ptr->camObj.size();

    // if (num_box > _max_num_box){
    //     num_box = _max_num_box;
    // }

    //
    size_t _box_count = 0;
    for (size_t i = 0; i < num_box; i++)
	{
        //
        auto & _box = msg_out_ptr->camObj[i];
        box_param_cv _a_box_param_cv(_box.x, _box.y, _box.width, _box.height, _box.cls);
        box_param_gl _a_box_param_gl;
        convert_cv_to_normalized_gl(_a_box_param_cv, _a_box_param_gl);
        if (!is_gl_box_valid(_a_box_param_gl)){
            continue; // Don't add to buffer
        }
        _box_count++;
        //
        glm::vec3 _box_color = rmlv2TagBoundingBox2D_ns::get_obj_class_color(_a_box_param_gl.obj_class);
        if (is_perspected){
            text2Din3D_list.emplace_back(
                "#" + std::to_string(_box.id) + " cls: " + std::to_string(_box.cls),
                _a_box_param_gl.xy_list[0],
                0.1,
                _box_color,
                rmText3D_v2::ALIGN_X::LEFT,
                rmText3D_v2::ALIGN_Y::BUTTON,
                1
            );
        }else{
            text2Dflat_list.emplace_back(
                "#" + std::to_string(_box.id) + " cls: " + std::to_string(_box.cls),
                _a_box_param_gl.xy_list[0],
                24,
                _box_color,
                rmText3D_v2::ALIGN_X::LEFT,
                rmText3D_v2::ALIGN_Y::BUTTON,
                1,
                0,
                !is_moveable,
                false
            );
        }
        //
	}

    // Insert texts
    if (is_perspected){
        rm_text.insert_text(text2Din3D_list);
    }else{
        rm_text.insert_text(text2Dflat_list);
    }
}
*/


void rmlv2TagBoundingBox2D::update_GL_data(){
    // Reset
    if (is_perspected){
        text2Din3D_list.clear();
    }else{
        text2Dflat_list.clear();
    }

    //
    if (msg_out_ptr->objects.size() == 0){
        // Insert texts
        if (is_perspected){
            rm_text.insert_text(text2Din3D_list);
        }else{
            rm_text.insert_text(text2Dflat_list);
        }
        return;
    }
    long long num_box = msg_out_ptr->objects.size();
    /*
    if (num_box > _max_num_box){
        num_box = _max_num_box;
    }
    */


    //
    size_t _box_count = 0;
    for (size_t i = 0; i < num_box; i++)
	{
        //
        auto & _box = msg_out_ptr->objects[i];
        box_param_cv _a_box_param_cv(_box.camInfo.x, _box.camInfo.y, _box.camInfo.width, _box.camInfo.height, _box.classId);
        box_param_gl _a_box_param_gl;
        convert_cv_to_normalized_gl(_a_box_param_cv, _a_box_param_gl);
        if (!is_gl_box_valid(_a_box_param_gl)){
            continue; // Don't add to buffer
        }
        _box_count++;
        //
        glm::vec3 _box_color = rmlv2TagBoundingBox2D_ns::get_obj_class_color(_a_box_param_gl.obj_class);
        if (is_perspected){
            text2Din3D_list.emplace_back(
                "#" + std::to_string(_box.id) + " cls: " + std::to_string(_box.cls),
                _a_box_param_gl.xy_list[0],
                0.1,
                _box_color,
                rmText3D_v2::ALIGN_X::LEFT,
                rmText3D_v2::ALIGN_Y::BUTTON,
                1
            );
        }else{
            text2Dflat_list.emplace_back(
                "#" + std::to_string(_box.id) + " cls: " + std::to_string(_box.cls),
                _a_box_param_gl.xy_list[0],
                24,
                _box_color,
                rmText3D_v2::ALIGN_X::LEFT,
                rmText3D_v2::ALIGN_Y::BUTTON,
                1,
                0,
                !is_moveable,
                false
            );
        }
        //
	}

    // Insert texts
    if (is_perspected){
        rm_text.insert_text(text2Din3D_list);
    }else{
        rm_text.insert_text(text2Dflat_list);
    }
}


void rmlv2TagBoundingBox2D::setBoardSize(float width_in, float height_in){
    rm_box.setBoardSize(width_in, height_in);
    rm_text.setBoardSize(width_in, height_in);
}
void rmlv2TagBoundingBox2D::setBoardSize(float size_in, bool is_width){ // Using the aspect ratio from pixel data
    rm_box.setBoardSize(size_in, is_width);
    rm_text.setBoardSize(size_in, is_width);
}
void rmlv2TagBoundingBox2D::setBoardSizeRatio(float ratio_in, bool is_width){ // Only use when is_perspected==false is_moveable==true
    rm_box.setBoardSize(ratio_in, is_width);
    rm_text.setBoardSize(ratio_in, is_width);
}
