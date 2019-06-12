#ifndef RM_BASEMODEL_H
#define RM_BASEMODEL_H



#include "Common.h"
#include "Shader.h"
#include "ViewManager.h"
//
#include <ROS_ICLU3_v0.hpp>


class rmBaseModel{


public:
	rmBaseModel();
    rmBaseModel(std::string _path_Assets_in);
    rmBaseModel(std::string _path_Assets_in, std::string modelFile, std::string textFile);
	~rmBaseModel();
    //
	virtual void Update(float dt);
    virtual void Update(ROS_INTERFACE &ros_interface);
	virtual void Render(std::shared_ptr<ViewManager> _camera_ptr);

    // Matrix operation
	void Translate(glm::vec3 vec);
	void Rotate(glm::vec3 axis, float angle);
	void Scale(glm::vec3 vec);
    glm::mat4 get_mv_matrix(std::shared_ptr<ViewManager> _camera_ptr, glm::mat4 &_model_M);
    // Utilities
    std::string get_full_Assets_path(std::string Assets_name_in);
    std::string get_full_Shader_path(std::string Shader_name_in);
    static glm::mat4 ROStf2GLMmatrix(const geometry_msgs::TransformStamped &ros_tf);

protected:

    std::string _path_Assets;
    std::string _path_Shaders;
	std::shared_ptr<ShaderProgram> _program_ptr;
    //
    glm::mat4 translateMatrix;
	glm::mat4 rotateMatrix;
	glm::mat4 scaleMatrix;
    //
	//
    virtual void Init();
    virtual void LoadModel();

private:
    // model info
    struct Shape{
        GLuint vao;
        GLuint vbo;
        GLuint vboTex;
        GLuint ebo;
        GLuint p_normal;
        GLuint m_texture;
        //
        int materialId;
        int indexCount;

        glm::mat4 model;
    };
    Shape m_shape;
    //
    std::string objName;
	std::string textName;


    //uniform id
	struct
	{
		GLint  mv_matrix;
		GLint  proj_matrix;
	} uniforms;

};


#endif  // RM_BASEMODEL_H
