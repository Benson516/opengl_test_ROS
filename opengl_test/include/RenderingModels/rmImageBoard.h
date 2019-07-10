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
    // Different alignment
    //--------------------------------------//
    enum class ALIGN_X{
        LEFT,
        CENTER,
        RIGHT
    };
    enum class ALIGN_Y{
        TOP,
        CENTER,
        BUTTON
    };
    //--------------------------------------//

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
    void Reshape(const glm::ivec2 & viewport_size_in);
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

    // Set 2D image (moveable) position
    void setBoardPositionCVPixel(
        int cv_x,
        int cv_y,
        int ref_point_mode_in=0,
        ALIGN_X     align_x_in=ALIGN_X::CENTER,
        ALIGN_Y     align_y_in=ALIGN_Y::CENTER
    );
    void updateBoardPosition();
    // ref_point_mode:
    // 0: upper-left corner
    // 1: upper-right corner
    // 2: lower-left corner
    // 3: lower-right corner


    inline void updateBoardGeo(){
        updateBoardSize(); // Do this first
        updateBoardPosition();
    }


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

    // Note: The origin of the image is at its center.
    int im_pixel_width;
    int im_pixel_height;
    float im_aspect; // w / h
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

    // Board position
    bool is_using_cv_pose;
    glm::ivec2 cv_pose;
    int ref_point_mode;
    ALIGN_X board_align_x;
    ALIGN_Y board_align_y;
    // ref_point_mode:
    // 0: upper-left corner
    // 1: upper-right corner
    // 2: lower-left corner
    // 3: lower-right corner

    void update_GL_data();


private:
    // model info
    struct Shape{
        GLuint vao;
        GLuint vbo;
        GLuint m_texture;
        // image
        // size_t width;
        // size_t height;
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
