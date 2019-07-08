#ifndef RM_POLY_LINES_3D_H
#define RM_POLY_LINES_3D_H

#include "rmBaseModel.h"

#include <queue>          // std::queue

class rmPolyLines3D : public rmBaseModel
{
public:

    // The structure for point
    struct circle_data
	{
		glm::vec3     position;
        float         radious;
		glm::vec3     color;
        circle_data(
            glm::vec3     position_in=glm::vec3(0.0f),
            float         radious_in=1.0f,
    		glm::vec3     color_in=glm::vec3(1.0f)
        ):
            position(position_in),
            radious(radious_in),
            color(color_in)
        {}
	};


    rmPolyLines3D(std::string _path_Assets_in);
    //
	void Update(float dt);
    void Update(ROS_INTERFACE &ros_interface);
    void Update(ROS_API &ros_api);
	void Render(std::shared_ptr<ViewManager> &_camera_ptr);

    // Insert method for circle
    //-------------------------------------//
    void insert_circle(const vector<circle_data> & data_list_in );
    //-------------------------------------//

protected:
    void Init();
    virtual void LoadModel();
    //
    // int _ROS_topic_id;
    std::shared_ptr< msgs::LidRoi > msg_out_ptr;
    // ros::Time msg_time;


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


    int _num_vertex_per_shape;
    long long _max_num_vertex;
    long long _max_num_shape;



    //uniform id
	struct
	{
		GLint  mv_matrix;
		GLint  proj_matrix;
	} uniforms;

    // buffer
    std::vector<circle_data> circle_buffer;


};

#endif // RM_POLY_LINES_3D_H
