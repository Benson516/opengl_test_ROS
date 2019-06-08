#include "BaseModel.h"


BaseModel::BaseModel(){
}

BaseModel::BaseModel(char* modelFile,char* textFile){
    _path_Assets = "./Assets/";
    _path_Shaders = _path_Assets + "Shaders/";
	objName = modelFile;
	textName = textFile;
	Init();
}
BaseModel::BaseModel(std::string _path_Assets_in, std::string modelFile, std::string textFile){
    _path_Assets = _path_Assets_in;
    _path_Shaders = _path_Assets + "Shaders/";
    objName = _path_Assets + modelFile;
    textName = _path_Assets + textFile;
	Init();
}

BaseModel::~BaseModel(){

}

void BaseModel::Init(){
	program = new ShaderProgram();
	program->CreateProgram();
	Shader* vs = new Shader();
	Shader* fs = new Shader();

    // Load shaders
    vs->LoadShader(get_full_shader_path("BaseModel.vs.glsl"), GL_VERTEX_SHADER);
	fs->LoadShader(get_full_shader_path("BaseModel.fs.glsl"), GL_FRAGMENT_SHADER);

	program->AttachShader(vs->GetID());
	program->AttachShader(fs->GetID());
	program->LinkProgram();

	//init model matrix
	m_shape.model = glm::mat4();
	translateMatrix = glm::mat4();
	rotateMatrix = glm::mat4();
	scaleMatrix = glm::mat4();

	//Cache uniform variable id
	uniforms.proj_matrix = glGetUniformLocation(program->GetID(), "um4p");
	uniforms.mv_matrix = glGetUniformLocation(program->GetID(), "um4mv");

	program->UseProgram();
	///////////////////////////

	//Load model to shader program
	LoadModel();
}

void BaseModel::LoadModel(){
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;

	std::string err;

    std::cout << "Start loading <" << objName << ">\n";
	bool ret = tinyobj::LoadObj(shapes, materials, err, objName.c_str());
	if (err.size()>0)
	{
		printf("Load Models Fail! Please check the solution path");
		return;
	}

    std::cout << "Load models success ! Shapes size = " <<  shapes.size() << ", Maerial size = " << materials.size() << "\n";

	/*
	*Bind model data
	*/
	for (int i = 0; i < shapes.size(); i++)
	{
		glGenVertexArrays(1, &m_shape.vao);
		glBindVertexArray(m_shape.vao);

		glGenBuffers(3, &m_shape.vbo);
		glGenBuffers(1, &m_shape.p_normal);
		glBindBuffer(GL_ARRAY_BUFFER, m_shape.vbo);
		glBufferData(GL_ARRAY_BUFFER, shapes[i].mesh.positions.size() * sizeof(float) + shapes[i].mesh.normals.size() * sizeof(float), NULL, GL_STATIC_DRAW);

		glBufferSubData(GL_ARRAY_BUFFER, 0, shapes[i].mesh.positions.size() * sizeof(float), &shapes[i].mesh.positions[0]);
		glBufferSubData(GL_ARRAY_BUFFER, shapes[i].mesh.positions.size() * sizeof(float), shapes[i].mesh.normals.size() * sizeof(float), &shapes[i].mesh.normals[0]);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, (void *)(shapes[i].mesh.positions.size() * sizeof(float)));

		glBindBuffer(GL_ARRAY_BUFFER, m_shape.p_normal);
		glBufferData(GL_ARRAY_BUFFER, shapes[i].mesh.normals.size() * sizeof(float), shapes[i].mesh.normals.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 0, 0);

		glBindBuffer(GL_ARRAY_BUFFER, m_shape.vboTex);
		glBufferData(GL_ARRAY_BUFFER, shapes[i].mesh.texcoords.size() * sizeof(float), shapes[i].mesh.texcoords.data(), GL_STATIC_DRAW);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_shape.ebo);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, shapes[i].mesh.indices.size() * sizeof(unsigned int), shapes[i].mesh.indices.data(), GL_STATIC_DRAW);
		m_shape.materialId = shapes[i].mesh.material_ids[0];
		m_shape.indexCount = shapes[i].mesh.indices.size();

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glEnableVertexAttribArray(2);
	}

	/*
	*Texture setting
	*/

	//Load texture data from file
    std::cout << "start loading <" << textName << "\n";
	TextureData tdata = Common::Load_png(textName.c_str());

	//Generate empty texture
	glGenTextures(1, &m_shape.m_texture);
	glBindTexture(GL_TEXTURE_2D, m_shape.m_texture);

	//Do texture setting
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, tdata.width, tdata.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, tdata.data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    std::cout << "Load texture success!\n";
}

void BaseModel::Update(float dt){
    // Update the data (uniform variables) here
}
void BaseModel::Render(ViewManager* _camera_ptr){
	//Update shaders' input variable
	///////////////////////////
	glBindVertexArray(m_shape.vao);
	program->UseProgram();

	m_shape.model = translateMatrix * rotateMatrix * scaleMatrix;
	glBindTexture(GL_TEXTURE_2D, m_shape.m_texture);
	glUniformMatrix4fv(uniforms.mv_matrix, 1, GL_FALSE, value_ptr(_camera_ptr->GetViewMatrix() * _camera_ptr->GetModelMatrix() * m_shape.model));
	glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, value_ptr(_camera_ptr->GetProjectionMatrix()));

	glDrawElements(GL_TRIANGLES, m_shape.indexCount, GL_UNSIGNED_INT, 0);
	///////////////////////////
}

void BaseModel::Translate(glm::vec3 vec){
	translateMatrix = translate(translateMatrix, vec);
}

void BaseModel::Rotate(glm::vec3 axis,float angle){
	rotateMatrix = rotate(rotateMatrix, angle, axis);
}

void BaseModel::Scale(glm::vec3 vec){
	scaleMatrix = scale(scaleMatrix, vec);
}
