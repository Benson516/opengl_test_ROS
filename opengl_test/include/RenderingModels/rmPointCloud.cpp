#include "rmPointCloud.h"


rmPointCloud::rmPointCloud(std::string _path_Assets_in){
    _path_Assets = _path_Assets_in;
    _path_Shaders = _path_Assets + "Shaders/";
    Init();
}
void rmPointCloud::Init(){
    //
	_program_ptr.reset( new ShaderProgram() );
    // Load shaders
    _program_ptr->AttachShader(get_full_Shader_path("PointCloud.vs.glsl"), GL_VERTEX_SHADER);
    _program_ptr->AttachShader(get_full_Shader_path("PointCloud.fs.glsl"), GL_FRAGMENT_SHADER);
    // Link _program_ptr
	_program_ptr->LinkProgram();
    //

}
void rmPointCloud::Update(float dt){

}
void rmPointCloud::Render(std::shared_ptr<ViewManager> _camera_ptr){


}
