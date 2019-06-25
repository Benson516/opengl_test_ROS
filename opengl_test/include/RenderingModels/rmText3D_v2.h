#ifndef RM_TEXT_3D_v2_H
#define RM_TEXT_3D_v2_H

#include "rmBaseModel.h"


#include <map> // std::map
// FreeType
#include <ft2build.h>
#include FT_FREETYPE_H



struct atlas;



class rmText3D_v2 : public rmBaseModel
{
public:
    rmText3D_v2(std::string _path_Assets_in, int _ROS_topic_id_in);
    //
	void Update(float dt);
    void Update(ROS_INTERFACE &ros_interface);
    void Update(ROS_API &ros_api);
	void Render(std::shared_ptr<ViewManager> _camera_ptr);

protected:
    void Init();
    virtual void LoadModel();
    //
    int _ROS_topic_id;
    // std::shared_ptr< msgs::LidRoi > box3d_out_ptr;
    // ros::Time msg_time;


    void RenderText(const std::string &text, atlas * _atlas_ptr, float x, float y, float scale_x_in, float scale_y_in, glm::vec3 color);

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
    std::string text_current;

    // Pointers of atlas
    atlas *a48;
    atlas *a24;
    atlas *a12;



};

#endif // RM_TEXT_3D_v2_H
