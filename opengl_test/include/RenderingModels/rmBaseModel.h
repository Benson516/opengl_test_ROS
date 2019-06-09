#ifndef RM_BASEMODEL_H
#define RM_BASEMODEL_H



#include "Common.h"
#include "Shader.h"
#include "ViewManager.h"


class rmBaseModel{


public:
	rmBaseModel();
    rmBaseModel(std::string _path_Assets_in);
    rmBaseModel(std::string _path_Assets_in, std::string modelFile, std::string textFile);
	~rmBaseModel();
    //
	virtual void Update(float dt);
	virtual void Render(std::shared_ptr<ViewManager> _camera_ptr);

    // Utilities
	void Translate(glm::vec3 vec);
	void Rotate(glm::vec3 axis, float angle);
	void Scale(glm::vec3 vec);
    std::string get_full_Assets_path(std::string Assets_name_in);
    std::string get_full_Shader_path(std::string Shader_name_in);

protected:

    std::string _path_Assets;
    std::string _path_Shaders;
	std::shared_ptr<ShaderProgram> _program_ptr;
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
        int materialId;
        int indexCount;
        GLuint m_texture;

        glm::mat4 model;
    };
    Shape m_shape;
    //
    std::string objName;
	std::string textName;
    //
    glm::mat4 translateMatrix;
	glm::mat4 rotateMatrix;
	glm::mat4 scaleMatrix;

    //uniform id
	struct
	{
		GLint  mv_matrix;
		GLint  proj_matrix;
	} uniforms;

};


#endif  // RM_BASEMODEL_H
