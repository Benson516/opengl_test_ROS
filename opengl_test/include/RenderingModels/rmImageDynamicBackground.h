#ifndef RM_IMAGE_DYNAMIC_BACKGROUND_H
#define RM_IMAGE_DYNAMIC_BACKGROUND_H

#include "rmBaseModel.h"





class rmImageDynamicBackground : public rmBaseModel
{
public:
    rmImageDynamicBackground(std::string _path_Assets_in, std::string image_file_in);
    rmImageDynamicBackground(std::string _path_Assets_in, int _ROS_topic_id_in);
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
    std::shared_ptr<cv::Mat> image_out_ptr;
    // ros::Time msg_time;

private:
    // model info
    struct Shape{
        GLuint vao;
        GLuint vbo;
        GLuint m_texture;
        //
        int indexCount;
        // image
        size_t width;
        size_t height;
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

#endif // RM_IMAGE_DYNAMIC_BACKGROUND_H
