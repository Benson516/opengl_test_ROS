#ifndef RM_LIDARBOUNDINGBOX_H
#define RM_LIDARBOUNDINGBOX_H

#include "rmBaseModel.h"

namespace rmLidarBoundingBox_ns{
    const GLuint box_idx_data[] = {
        0,1,2,
        2,3,0,
        5,6,2,
        2,1,5,
        3,2,6,
        6,7,3,
        4,5,1,
        1,0,4,
        6,5,4,
        4,7,6,
        4,0,3,
        3,7,4,
    };
}

class rmLidarBoundingBox : public rmBaseModel
{
public:
    rmLidarBoundingBox(std::string _path_Assets_in, int _ROS_topic_id_in);
    //
	void Update(float dt);
    void Update(ROS_INTERFACE &ros_interface);
	void Render(std::shared_ptr<ViewManager> _camera_ptr);

protected:
    void Init();
    virtual void LoadModel();
    //
    int _ROS_topic_id;
    std::shared_ptr< msgs::LidRoi > box3d_out_ptr;
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

    // The structure for point
    struct vertex_p_c
	{
		glm::vec3     position;
		glm::vec3     color;
	};
    int _num_vertex_idx_per_box;
    long long _max_num_vertex_idx;
    int _num_vertex_per_box;
    long long _max_num_vertex;
    long long _max_num_box;



    // The box
    float box_half_size = 1.0f;

    //uniform id
	struct
	{
		GLint  mv_matrix;
		GLint  proj_matrix;
	} uniforms;

    std::vector<vertex_p_c> box_template;

    inline void generate_box_template(){
        box_template.resize(8);
        box_template[0].position = glm::vec3(0.0f, 0.0f, 0.0f);
        box_template[1].position = glm::vec3(0.0f, 0.0f, 1.0f);
        box_template[2].position = glm::vec3(1.0f, 0.0f, 1.0f);
        box_template[3].position = glm::vec3(1.0f, 0.0f, 0.0f);
        box_template[4].position = glm::vec3(0.0f, 1.0f, 0.0f);
        box_template[5].position = glm::vec3(0.0f, 1.0f, 1.0f);
        box_template[6].position = glm::vec3(1.0f, 1.0f, 1.0f);
        box_template[7].position = glm::vec3(1.0f, 1.0f, 0.0f);
    }

};

#endif // RM_LIDARBOUNDINGBOX_H
