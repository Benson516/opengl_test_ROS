#ifndef RM_IMAGE_BOARD_H
#define RM_IMAGE_BOARD_H

#include "rmBaseModel.h"

/*

bool is_perspected_in=true,
bool is_moveable_in=true,
bool is_color_transformed_in=false


This is a combined model for all legacy image boards
- rmImageStatic             --> image_file_in,      true, (don't care), false
- rmImageDynamic            --> _ROS_topic_id_in,   true, (don't care), false
- rmImageStaticBackground   --> image_file_in,      false, false, true
- rmImageDynamicBackground  --> _ROS_topic_id_in,   false, false, true

We also got some new combinations
    - (any), false, true, true     <-- Flat image which can move/scaled/rotated, also possible to move to topest/lowest layer

Note:
    - For (is_perspected_in=false, is_moveable_in=true),  The image is set to top layer, (Front board)
    - For (is_perspected_in=false, is_moveable_in=false), The image is set to low layer, (background)
*/



class rmImageBoard : public rmBaseModel
{
public:
    rmImageBoard(
        std::string _path_Assets_in,
        std::string image_file_in,
        bool is_perspected_in=true,
        bool is_moveable_in=true,
        bool is_color_transformed_in=false
    );
    rmImageBoard(
        std::string _path_Assets_in,
        int _ROS_topic_id_in,
        bool is_perspected_in=true,
        bool is_moveable_in=true,
        bool is_color_transformed_in=false
    );
    //
	void Update(float dt);
    void Update(ROS_INTERFACE &ros_interface);
	void Render(std::shared_ptr<ViewManager> _camera_ptr);

    // Color transform
    float _alpha;
    glm::vec4 _color_transform;

protected:
    void Init();
    virtual void LoadModel();
    //
    int _ROS_topic_id;
    std::shared_ptr<cv::Mat> image_out_ptr;
    // ros::Time msg_time;

    // Settings
    bool is_perspected;
    bool is_moveable;
    bool is_color_transformed;
    bool is_dynamically_updated;

private:
    // model info
    struct Shape{
        GLuint vao;
        GLuint vbo;
        GLuint m_texture;
        // image
        size_t width;
        size_t height;
        //
        glm::mat4 model;
    };
    Shape m_shape;

    //
    std::string textName;


    //uniform id
	struct
	{
		GLint  mv_matrix;
		GLint  proj_matrix;
        //
        GLint  color_transform;
		GLint  alpha;
	} uniforms;


};

#endif // RM_IMAGE_BOARD_H