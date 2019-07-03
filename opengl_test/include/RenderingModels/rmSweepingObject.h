#ifndef RM_SWEEPING_OGJECT_H
#define RM_SWEEPING_OGJECT_H

#include "rmBaseModel.h"



class rmSweepingObject : public rmBaseModel
{
public:
    rmSweepingObject(std::string _path_Assets_in);
    rmSweepingObject(std::string _path_Assets_in, int _ROS_topic_id_in);
    //
	void Update(float dt);
    void Update(ROS_INTERFACE &ros_interface);
    void Update(ROS_API &ros_api);
	void Render(std::shared_ptr<ViewManager> &_camera_ptr);

protected:
    void Init();
    virtual void LoadModel();
    //
    int _ROS_topic_id;
    // std::shared_ptr< msgs::LidRoi > msg_out_ptr;
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
		glm::vec3     color;
	};

    //
    int _max_num_vertex_of_curve;
    // std::vector<float> _curve_Points;
    std::vector<glm::vec3> _curve_Points;

    // lookat matrix - a list of transformation matrices for each transaction
    std::vector<glm::mat4> lookat_matrix_list;


    int _max_num_vertex_of_shape;
    std::vector<glm::vec3> section_vertexes;

    //uniform id
	struct
	{
		GLint  mv_matrix;
		GLint  proj_matrix;
        std::vector<GLint> lookat_matrix;
        std::vector<GLint> section_vertexes;
        GLint _num_vertex_of_shape;
	} uniforms;

    std::vector<vertex_p_c> box_template;


};

#endif // RM_SWEEPING_OGJECT_H
