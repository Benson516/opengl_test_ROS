#include "rmBaseModel.h"


rmPointCloud::rmPointCloud((std::string _path_Assets_in){
    _path_Assets = _path_Assets_in;
    _path_Shaders = _path_Assets + "Shaders/";
    Init();
}
void rmPointCloud::Init(){

}
void rmPointCloud::Update(float dt){

}
void rmPointCloud::Render(std::shared_ptr<ViewManager> _camera_ptr){


}
