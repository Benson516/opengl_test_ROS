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

    //
    struct text2D_data{
        std::string text;
        glm::vec2   position_2D;
        glm::vec3   color;
        text2D_data(const std::string &text_in, const glm::vec2 &position_2D_in=glm::vec2(0.0f), const glm::vec3 &color_in=glm::vec3(1.0f) ):
            text(text_in), position_2D(position_2D_in), color(color_in)
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


    inline void insert_text(const std::string &text_in, const glm::vec2 &position_2D_in=glm::vec2(0.0f), const glm::vec3 &color_in=glm::vec3(1.0f) ){
        text_buffer.push( text2D_data(text_in, position_2D_in, color_in) );
    }

protected:
    void Init();
    virtual void LoadModel();
    //
    int _ROS_topic_id;
    // std::shared_ptr< msgs::LidRoi > box3d_out_ptr;
    // ros::Time msg_time;

    // void RenderText(const std::string &text, atlas *_atlas_ptr, float x, float y, float scale_x_in, float scale_y_in, glm::vec3 color);
    void RenderText(const std::string &text, std::shared_ptr<atlas> &_atlas_ptr, float x_in, float y_in, float scale_x_in, float scale_y_in, glm::vec3 color);
    void _draw_one_text(std::shared_ptr<ViewManager> &_camera_ptr, text2D_data &_data_in);

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

    std::queue<text2D_data> text_buffer;



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
