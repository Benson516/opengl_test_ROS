#ifndef RM_BASEMODEL_H
#define RM_BASEMODEL_H



#include "Common.h"
#include "Shader.h"
// #include "ViewManager.h"
#include "ViewManager_v2.h"
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
    virtual void Update(ROS_API &ros_api);
	virtual void Render(std::shared_ptr<ViewManager> _camera_ptr);

    // Matrix operation
    //------------------------------------------------//
    // Legacy "Pre-" operations
    void Translate(const glm::vec3 &vec);
	void Rotate(const glm::vec3 &axis, float angle);
	void Scale(const glm::vec3 &vec);
    // "Pre-" operations
    void preTranslate(const glm::vec3 &vec);
	void preRotate(const glm::vec3 &axis, float angle);
	void preScale(const glm::vec3 &vec);
    // "Post-" operations
    void postTranslate(const glm::vec3 &vec);
	void postRotate(const glm::vec3 &axis, float angle);
	void postScale(const glm::vec3 &vec);
    // Directly set the model matrix
    virtual void setModelMatrix(const glm::mat4 & model_m_in){ *_pose_model_by_model_ref_ptr = model_m_in;  }
    //------------------------------------------------//

    // The pose
    inline void attach_pose_model_by_model_ref_ptr(glm::mat4 &pose_in){_pose_model_by_model_ref_ptr = &pose_in;}
    void set_pose_modle_ref_by_world(glm::mat4 pose_in);
    glm::mat4 get_pose_modle_ref_by_world();
    //
    glm::mat4 get_mv_matrix(std::shared_ptr<ViewManager> _camera_ptr, glm::mat4 &_model_M);


    // Utilities
    bool init_paths(std::string _path_Assets_in);
    std::string get_full_Assets_path(std::string Assets_name_in);
    std::string get_full_Shader_path(std::string Shader_name_in);
    static glm::mat4 ROStf2GLMmatrix(const geometry_msgs::TransformStamped &ros_tf);

    TIME_STAMP::FPS fps_of_update;
protected:
    //
    std::string _path_Assets_sub_dir;
    std::string _path_Shaders_sub_dir;
    //
    std::string _path_Assets;
    std::string _path_Shaders;
	std::shared_ptr<ShaderProgram> _program_ptr;
    //
    glm::mat4 translateMatrix;
	glm::mat4 rotateMatrix;
	glm::mat4 scaleMatrix;
    //
    glm::mat4 * _pose_model_by_model_ref_ptr;
    glm::mat4 _tmp_pose_model_by_model_ref;
    glm::mat4 _pose_modle_ref_by_world;
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

        glm::mat4 model; // Note: this is the pose relative to the mode reference frame, not the world frame
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
