#ifndef RM_TEXT_3D_v2_H
#define RM_TEXT_3D_v2_H

#include "rmBaseModel.h"

#include <queue>          // std::queue


// #include <map> // std::map
// FreeType
#include <ft2build.h>
#include FT_FREETYPE_H








// atlas
struct atlas;
//


class rmText3D_v2 : public rmBaseModel
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


    // Different drawing method
    //--------------------------------------//
    // text2D in 3D space
    struct text2D_data{
        std::string text;
        glm::vec2   position_2D;
        float       size; // The "hight of line" in meter
        glm::vec3   color;
        ALIGN_X     align_x;
        ALIGN_Y     align_y;
        text2D_data(
            const std::string &text_in,
            const glm::vec2 &position_2D_in=glm::vec2(0.0f),
            float size_in=1.0f,
            const glm::vec3 &color_in=glm::vec3(1.0f),
            ALIGN_X align_x_in=ALIGN_X::LEFT,
            ALIGN_Y align_y_in=ALIGN_Y::TOP
        ):
            text(text_in),
            position_2D(position_2D_in),
            size(size_in),
            color(color_in),
            align_x(align_x_in),
            align_y(align_y_in)
        {
        }
    };
    // text3D as an object in space
    struct text3D_data{// Different drawing method
        std::string text;
        glm::mat4   pose_ref_point;
        glm::vec2   offset_ref_point_2D;
        float       size; // The "hight of line" in meter
        glm::vec3   color;
        ALIGN_X     align_x;
        ALIGN_Y     align_y;
        text3D_data(
            const std::string &text_in,
            const glm::mat4 &pose_ref_point_in=glm::mat4(1.0f),
            const glm::vec2 &offset_ref_point_2D_in=glm::vec2(0.0f),
            float size_in=1.0f,
            const glm::vec3 &color_in=glm::vec3(1.0f),
            ALIGN_X align_x_in=ALIGN_X::LEFT,
            ALIGN_Y align_y_in=ALIGN_Y::TOP
        ):
            text(text_in),
            pose_ref_point(pose_ref_point_in),
            offset_ref_point_2D(offset_ref_point_2D_in),
            size(size_in),
            color(color_in),
            align_x(align_x_in),
            align_y(align_y_in)
        {}
    };
    // text3D as a billboard attached to a 3D point
    struct text_billboard_data{
        std::string text;
        glm::vec3   position_ref_point;
        glm::vec2   offset_ref_point_2D;
        float       size; // The "hight of line" in meter
        glm::vec3   color;
        ALIGN_X     align_x;
        ALIGN_Y     align_y;
        text_billboard_data(
            const std::string &text_in,
            const glm::vec3 &position_ref_point_in=glm::vec3(0.0f),
            const glm::vec2 &offset_ref_point_2D_in=glm::vec2(0.0f),
            float size_in=1.0f,
            const glm::vec3 &color_in=glm::vec3(1.0f),
            ALIGN_X align_x_in=ALIGN_X::LEFT,
            ALIGN_Y align_y_in=ALIGN_Y::TOP
        ):
            text(text_in),
            position_ref_point(position_ref_point_in),
            offset_ref_point_2D(offset_ref_point_2D_in),
            size(size_in),
            color(color_in),
            align_x(align_x_in),
            align_y(align_y_in)
        {}
    };
    // text3D as a billboard attached to a 3D point with fixed size
    struct text_freeze_board_data{
        std::string text;
        glm::vec3   position_ref_point;
        glm::vec2   offset_ref_point_2D;
        float       size; // The "hight of line" in "pixel"
        glm::vec3   color;
        ALIGN_X     align_x;
        ALIGN_Y     align_y;
        text_freeze_board_data(
            const std::string &text_in,
            const glm::vec3 &position_ref_point_in=glm::vec3(0.0f),
            const glm::vec2 &offset_ref_point_2D_in=glm::vec2(0.0f),
            float size_in=48,
            const glm::vec3 &color_in=glm::vec3(1.0f),
            ALIGN_X align_x_in=ALIGN_X::LEFT,
            ALIGN_Y align_y_in=ALIGN_Y::TOP
        ):
            text(text_in),
            position_ref_point(position_ref_point_in),
            offset_ref_point_2D(offset_ref_point_2D_in),
            size(size_in),
            color(color_in),
            align_x(align_x_in),
            align_y(align_y_in)
        {}
    };
    //--------------------------------------//
    // end Different drawing method







    rmText3D_v2(std::string _path_Assets_in);
    //
	void Update(float dt);
    void Update(ROS_INTERFACE &ros_interface);
    void Update(ROS_API &ros_api);
	void Render(std::shared_ptr<ViewManager> &_camera_ptr);

    // Insert method for texts
    // queues - draw once
    //-------------------------------------//
    inline void insert_text(const text2D_data & data_in ){
        text2D_queue.push( data_in );
    }
    inline void insert_text(const text3D_data & data_in ){
        text3D_queue.push( data_in );
    }
    inline void insert_text(const text_billboard_data & data_in ){
        text_billboard_queue.push( data_in );
    }
    inline void insert_text(const text_freeze_board_data & data_in ){
        text_freeze_board_queue.push( data_in );
    }
    //-------------------------------------//

    // buffers - draw each time until update
    //-------------------------------------//
    inline void insert_text(const std::vector<text2D_data> & data_list_in ){
        text2D_buffer = data_list_in;
    }
    inline void insert_text(const std::vector<text3D_data> & data_list_in ){
        text3D_buffer = data_list_in;
    }
    inline void insert_text(const std::vector<text_billboard_data> & data_list_in ){
        text_billboard_buffer = data_list_in;
    }
    inline void insert_text(const std::vector<text_freeze_board_data> & data_list_in ){
        text_freeze_board_buffer = data_list_in;
    }
    //-------------------------------------//


protected:
    void Init();
    virtual void LoadModel();
    //
    int _ROS_topic_id;
    // std::shared_ptr< msgs::LidRoi > box3d_out_ptr;
    // ros::Time msg_time;

    // void RenderText(const std::string &text, atlas *_atlas_ptr, float x, float y, float scale_x_in, float scale_y_in, glm::vec3 color);
    void RenderText(
        const std::string &text,
        std::shared_ptr<atlas> &_atlas_ptr,
        float x_in,
        float y_in,
        float scale_x_in,
        float scale_y_in,
        glm::vec3 color,
        ALIGN_X align_x=ALIGN_X::LEFT,
        ALIGN_Y align_y=ALIGN_Y::TOP
    );

    // Different draw methods
    //--------------------------------------------------------//
    void _draw_one_text2D(std::shared_ptr<ViewManager> &_camera_ptr, text2D_data &_data_in);
    void _draw_one_text3D(std::shared_ptr<ViewManager> &_camera_ptr, text3D_data &_data_in);
    void _draw_one_text_billboard(std::shared_ptr<ViewManager> &_camera_ptr, text_billboard_data &_data_in);
    void _draw_one_text_freeze_board(std::shared_ptr<ViewManager> &_camera_ptr, text_freeze_board_data &_data_in);
    //--------------------------------------------------------//

private:
    // model info
    struct Shape{
        GLuint vao;
        GLuint vbo;
        GLuint ebo;
        GLuint m_texture;
        //
        int indexCount;

        glm::mat4 model;
    };
    Shape m_shape;


    struct point {
    	GLfloat x;
    	GLfloat y;
    	GLfloat s;
    	GLfloat t;
    };

    //uniform id
	struct
	{
		GLint  mv_matrix;
		GLint  proj_matrix;
        GLint  textColor;
        GLint  ref_point;
	} uniforms;

    glm::vec2 ref_point;

    //
    int _num_vertex_idx_per_box;
    long long _max_num_vertex_idx;
    int _num_vertex_per_box;
    long long _max_num_vertex;
    long long _max_num_box;
    // long long _max_string_length;


    //
    //------------------------------------------//
    // queues - only draw on time
    std::queue<text2D_data> text2D_queue;
    std::queue<text3D_data> text3D_queue;
    std::queue<text_billboard_data> text_billboard_queue;
    std::queue<text_freeze_board_data> text_freeze_board_queue;
    // buffers - draw each time
    std::vector<text2D_data> text2D_buffer;
    std::vector<text3D_data> text3D_buffer;
    std::vector<text_billboard_data> text_billboard_buffer;
    std::vector<text_freeze_board_data> text_freeze_board_buffer;
    //------------------------------------------//



    // Pointers of atlas
    /*
    atlas *a48_ptr;
    atlas *a24_ptr;
    atlas *a12_ptr;
    */
    /*
    std::shared_ptr<atlas> a48_ptr;
    std::shared_ptr<atlas> a24_ptr;
    std::shared_ptr<atlas> a12_ptr;
    */



};

#endif // RM_TEXT_3D_v2_H
