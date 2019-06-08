#ifndef BASEMODEL_H
#define BASEMODEL_H



#include "Common.h"
#include "Shader.h"
#include "ViewManager.h"


class BaseModel{

public:
    //model info
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
    //uniform id
	struct
	{
		GLint  mv_matrix;
		GLint  proj_matrix;
	} uniforms;


	BaseModel();
	BaseModel(char* modelFile,char* textFile);
    BaseModel(std::string _path_Assets_in, std::string modelFile, std::string textFile);
	~BaseModel();
	virtual void Init();
	virtual void Update(float dt);
	virtual void Render(ViewManager* _camera_ptr);

	void Translate(glm::vec3 vec);
	void Rotate(glm::vec3 axis, float angle);
	void Scale(glm::vec3 vec);
    std::string get_full_shader_path(std::string shader_name_in){
        std::string s_p;
        s_p = _path_Shaders + shader_name_in;
        std::cout << "shader = <" << s_p << ">\n";
        return s_p;
    }

protected:


    std::string _path_Assets;
    std::string _path_Shaders;
	std::string objName;
	std::string textName;
	ShaderProgram* program;
	Shape m_shape;
	virtual void LoadModel();
	glm::mat4 translateMatrix;
	glm::mat4 rotateMatrix;
	glm::mat4 scaleMatrix;

private:
    //

};


#endif  // BASEMODEL_H
