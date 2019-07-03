#ifndef RM_CIRCLE_H
#define RM_CIRCLE_H

#include "rmBaseModel.h"



class rmCircle : public rmBaseModel
{
public:
    rmCircle(std::string _path_Assets_in, int _ROS_topic_id_in);
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
    std::shared_ptr< msgs::LidRoi > msg_out_ptr;
    // ros::Time msg_time;

    void update_GL_data();

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

    // The structure for point
    struct vertex_p_c
	{
		glm::vec3     position;
        float         radious;
		glm::vec3     color;
	};
    int _num_vertex_per_box;
    long long _max_num_vertex;
    long long _max_num_box;



    //uniform id
	struct
	{
		GLint  mv_matrix;
		GLint  proj_matrix;
	} uniforms;



};

#endif // RM_CIRCLE_H
