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
    void Update(ROS_API &ros_api);
	void Render(std::shared_ptr<ViewManager> &_camera_ptr);
    //
    inline glm::mat4 * get_model_m_ptr(){ return &(m_shape.model); }

    // Color transform
    float alpha;
    glm::vec4 color_transform;

    // Set board size
    void setBoardSize(float width_in, float height_in); // 3D space
    void setBoardSize(float size_in, bool is_width); // 3D space / Using the aspect ratio from pixel data
    void setBoardSizeRatio(float ratio_in, bool is_width); // Only use when is_perspected==false is_moveable==true
    void updateBoardSize();
    // TIME_STAMP::FPS fps_of_update;

protected:
    void Init();
    virtual void LoadModel();
    //
    int _ROS_topic_id;
    std::shared_ptr<cv::Mat> msg_out_ptr;
    // ros::Time msg_time;

    // Settings
    bool is_perspected;
    bool is_moveable;
    bool is_color_transformed;
    bool is_dynamically_updated;

    // Params
    float board_width; // meter
    float board_height; // meter
    float board_aspect_ratio; // w/h
    int board_shape_mode;
    glm::ivec2 _viewport_size; // (w,h)
    // mode:
    // 0 - fixed size
    // 1 - fixed width
    // 2 - fixed height
    // 3 - fixed width ratio relative to viewport
    // 4 - fixed height ratio ralative to viewport

    void update_GL_data();


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
        glm::mat4 shape;
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
