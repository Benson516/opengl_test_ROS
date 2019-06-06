#ifndef BASEMODEL_H
#define BASEMODEL_H

#include <iostream>
#include <string>
#include "Common.h"
#include "Shader.h"



class BaseModel{
    
public:
	//uniform id
	struct
	{
		GLint  mv_matrix;
		GLint  proj_matrix;
	} uniforms;

	//model info
	typedef struct
	{
		GLuint vao;
		GLuint vbo;
		GLuint vboTex;
		GLuint ebo;

		GLuint p_normal;
		int materialId;
		int indexCount;
		GLuint m_texture;

		glm::mat4 model;
	} Shape;



	BaseModel();
	BaseModel(char* modelFile,char* textFile);
    BaseModel(std::string path_in, std::string modelFile, std::string textFile);
	~BaseModel();
	virtual void Init();
	virtual void Update(float dt);
	virtual void Render();

	void Translate(glm::vec3 vec);
	void Rotate(glm::vec3 axis, float angle);
	void Scale(glm::vec3 vec);

protected:
    std::string _path;
	std::string objName;
	std::string textName;
	ShaderProgram* program;
	Shape m_shape;
	virtual void LoadModel();
	glm::mat4 translateMatrix;
	glm::mat4 rotateMatrix;
	glm::mat4 scaleMatrix;

private:

};

#endif  // BASEMODEL_H
