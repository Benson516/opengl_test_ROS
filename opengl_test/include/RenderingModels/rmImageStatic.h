#ifndef RM_IMAGE_STATIC_H
#define RM_IMAGE_STATIC_H

#include "rmBaseModel.h"





class rmImageStatic : public rmBaseModel
{
public:
    rmImageStatic(std::string _path_Assets_in, std::string image_file_in);
    rmImageStatic(std::string _path_Assets_in, int _ROS_topic_id_in);
    //
	void Update(float dt);
    void Update(ROS_INTERFACE &ros_interface);
	void Render(std::shared_ptr<ViewManager> _camera_ptr);

protected:
    void Init();
    virtual void LoadModel();
    //
    int _ROS_topic_id;
    // ros::Time msg_time;

private:
    // model info
    struct Shape{
        GLuint vao;
        GLuint vbo;
        GLuint m_texture;
        //
        int indexCount;

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
	} uniforms;


};

#endif // RM_IMAGE_STATIC_H
