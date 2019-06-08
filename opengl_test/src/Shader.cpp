#include "Shader.h"


void Shader::LoadShader(const char* fileName, int shaderType){
    std::cout << "Start loading shader\n";
	GLuint s = glCreateShader(shaderType);
	char** source = Common::LoadShaderSource(fileName);
	glShaderSource(s, 1, source, NULL);
	Common::FreeShaderSource(source);
	glCompileShader(s);

	id = s;
	type = shaderType;
	loaded = true;
}
void Shader::LoadShader(std::string fileName, int shaderType){
    LoadShader(fileName.c_str(), shaderType);
}

void Shader::Delete(){
	if (!loaded)return;
	loaded = false;
	glDeleteShader(id);
}

ShaderProgram::ShaderProgram()
{
	linked = false;
}

GLuint Shader::GetID(){
	return id;
}

bool Shader::isLoaded(){
	return loaded;
}

GLuint ShaderProgram::CreateProgram(){
	id = glCreateProgram();
	return id;
}

GLuint ShaderProgram::GetID(){
	return id;
}

void ShaderProgram::Delete(){
	if (!linked)return;
	linked = false;
	glDeleteProgram(id);
}
void ShaderProgram::LinkProgram(){
	glLinkProgram(id);
	int iLinkStatus;
	glGetProgramiv(id, GL_LINK_STATUS, &iLinkStatus);
	linked = iLinkStatus == GL_TRUE;
}

void ShaderProgram::UseProgram(){
	if (linked)glUseProgram(id);
}

void ShaderProgram::AttachShader(GLuint shaderId){
	glAttachShader(id, shaderId);
}
