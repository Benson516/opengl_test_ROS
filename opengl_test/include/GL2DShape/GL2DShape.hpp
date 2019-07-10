#ifndef GL_2D_SHAPE_H
#define GL_2D_SHAPE_H

#include "Common.h"

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


// Size mode:
// 0 - fixed size
// 1 - fixed width
// 2 - fixed height
// 3 - (2D) fixed width ratio relative to viewport
// 4 - (2D) fixed height ratio ralative to viewport
// 5 - (2D) fixed pixel size
// 6 - (2D) fixed pixel width
// 7 - (2D) fixed pixel height

// ref_point_mode:
// 0: upper-left corner
// 1: upper-right corner
// 2: lower-left corner
// 3: lower-right corner

class GL2DShape{
public:


    GL2DShape(glm::vec2 original_board_size_in=glm::vec2(2.0f, 2.0f) );

    // Set board size
    void setBoardSize(float width_in, float height_in); // 3D space
    void setBoardSize(float size_in, bool is_width); // 3D space / Using the aspect ratio from pixel data
    void setBoardSizeRatio(float ratio_in, bool is_width); // Only use when is_perspected==false is_moveable==true
    void setBoardSizePixel(int px_width_in, int px_heighth_in);
    void setBoardSizePixel(int pixel_in, bool is_width);
    // Set 2D image (moveable) position
    void setBoardPositionCVPixel(
        int cv_x,
        int cv_y,
        int ref_point_mode_in=0,
        ALIGN_X     align_x_in=ALIGN_X::CENTER,
        ALIGN_Y     align_y_in=ALIGN_Y::CENTER
    );
    // ref_point_mode:
    // 0: upper-left corner
    // 1: upper-right corner
    // 2: lower-left corner
    // 3: lower-right corner

    // Update method
    void updateBoardGeo(const glm::ivec2 &viewportsize_in, float aspect_ratio_in=1.0f);

    // Getting methods
    inline bool get_shape(glm::mat4 &shape_out){ shape_out = _shape;    return true; }
    inline bool get_tranlate(glm::mat4 & translation_m_out){
        if (is_using_cv_pose){
            translation_m_out = _translation_m;
            return true;
        }
        return false;
    }

private:

    glm::mat4 _shape;
    glm::mat4 _translation_m;

    void updateBoardSize();
    void updateBoardPosition();

    //
    glm::vec2 original_board_size;

    // Note: The origin of the image is at its center.
    // int im_pixel_width;
    // int im_pixel_height;
    // float im_aspect; // w / h
    // Params
    float board_width; // meter or pixel
    float board_height; // meter or pixel
    float board_aspect_ratio; // w/h
    float board_size_ratio; // Only for mode 3 and 4
    int board_shape_mode;
    glm::ivec2 _viewport_size; // (w,h)
    // Size mode:
    // 0 - fixed size
    // 1 - fixed width
    // 2 - fixed height
    // 3 - (2D) fixed width ratio relative to viewport
    // 4 - (2D) fixed height ratio ralative to viewport
    // 5 - (2D) fixed pixel size
    // 6 - (2D) fixed pixel width
    // 7 - (2D) fixed pixel height

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

};

#endif // GL_2D_SHAPE_H
