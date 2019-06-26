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

    // text2D in 3D space
    struct text2D_data{
        std::string text;
        glm::vec2   position_2D;
        glm::vec3   color;
        text2D_data(
            const std::string &text_in,
            const glm::vec2 &position_2D_in=glm::vec2(0.0f),
            const glm::vec3 &color_in=glm::vec3(1.0f)
        ):
            text(text_in), position_2D(position_2D_in), color(color_in)
        {}
    };
    // text3D as an object in space
    struct text3D_data{
        std::string text;
        glm::mat4   pose_ref_point;
        glm::vec2   offset_ref_point_2D;
        glm::vec3   color;
        text3D_data(
            const std::string &text_in,
            const glm::mat4 &pose_ref_point_in=glm::mat4(1.0f),
            const glm::vec2 &offset_ref_point_2D_in=glm::vec2(0.0f),
            const glm::vec3 &color_in=glm::vec3(1.0f)
        ):
            text(text_in),
            pose_ref_point(pose_ref_point_in),
            offset_ref_point_2D(offset_ref_point_2D_in),
            color(color_in)
        {}
    };
    // text3D as a billboard attached to a 3D point
    struct text_billboard_data{
        std::string text;
        glm::vec3   position_ref_point;
        glm::vec2   offset_ref_point_2D;
        glm::vec3   color;
        text_billboard_data(
            const std::string &text_in,
            const glm::vec3 &position_ref_point_in=glm::vec3(0.0f),
            const glm::vec2 &offset_ref_point_2D_in=glm::vec2(0.0f),
            const glm::vec3 &color_in=glm::vec3(1.0f)
        ):
            text(text_in),
            position_ref_point(position_ref_point_in),
            offset_ref_point_2D(offset_ref_point_2D_in),
            color(color_in)
        {}
    };
    // text3D as a billboard attached to a 3D point with fixed size
    struct text_freeze_board_data{
        std::string text;
        glm::vec3   position_ref_point;
        glm::vec2   offset_ref_point_2D;
        glm::vec3   color;
        text_freeze_board_data(
            const std::string &text_in,
            const glm::vec3 &position_ref_point_in=glm::vec3(0.0f),
            const glm::vec2 &offset_ref_point_2D_in=glm::vec2(0.0f),
            const glm::vec3 &color_in=glm::vec3(1.0f)
        ):
            text(text_in),
            position_ref_point(position_ref_point_in),
            offset_ref_point_2D(offset_ref_point_2D_in),
            color(color_in)
        {}
    };
    //

public:
    rmText3D_v2(std::string _path_Assets_in, int _ROS_topic_id_in);
    //
	void Update(float dt);
    void Update(ROS_INTERFACE &ros_interface);
    void Update(ROS_API &ros_api);
	void Render(std::shared_ptr<ViewManager> _camera_ptr);

    // Insert method for texts
    //-------------------------------------//
    inline void insert_text(const text2D_data & data_in ){
        text2D_buffer.push( data_in );
    }
    inline void insert_text(const text3D_data & data_in ){
        text3D_buffer.push( data_in );
    }
    inline void insert_text(const text_billboard_data & data_in ){
        text_billboard_buffer.push( data_in );
    }
    inline void insert_text(const text_freeze_board_data & data_in ){
        text_freeze_board_buffer.push( data_in );
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
    void RenderText(const std::string &text, std::shared_ptr<atlas> &_atlas_ptr, float x_in, float y_in, float scale_x_in, float scale_y_in, glm::vec3 color);

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
	} uniforms;

    long long _max_string_length;


    //

    std::queue<text2D_data> text2D_buffer;
    std::queue<text3D_data> text3D_buffer;
    std::queue<text_billboard_data> text_billboard_buffer;
    std::queue<text_freeze_board_data> text_freeze_board_buffer;



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
