#ifndef RM_IMAGE_STATIC_BACK_GROUND_H
#define RM_IMAGE_STATIC_BACK_GROUND_H

#include "rmBaseModel.h"





class rmImageStaticBackGround : public rmBaseModel
{
public:
    rmImageStaticBackGround(std::string _path_Assets_in, std::string image_file_in);
    rmImageStaticBackGround(std::string _path_Assets_in, int _ROS_topic_id_in);
    //
	void Update(float dt);
    void Update(ROS_INTERFACE &ros_interface);
	void Render(std::shared_ptr<ViewManager> _camera_ptr);

    // Color transform
    float _alpha;
    glm::vec4 _color_transform;

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
    };
    Shape m_shape;

    //
    std::string textName;

    //



    //uniform id
	struct
	{
		GLint  color_transform;
		GLint  alpha;
	} uniforms;


};

#endif // RM_IMAGE_STATIC_BACK_GROUND_H
