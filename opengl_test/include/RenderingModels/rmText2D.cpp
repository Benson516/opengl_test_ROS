#include "rmText2D.h"


void *font2D = GLUT_BITMAP_TIMES_ROMAN_24;
void *fonts2D[] =
{
  GLUT_BITMAP_9_BY_15,
  GLUT_BITMAP_TIMES_ROMAN_10,
  GLUT_BITMAP_TIMES_ROMAN_24
};


rmText2D::rmText2D()
{
    // init_paths(_path_Assets_in);
	Init();
}
void rmText2D::Init(){
    //
    // Current text
    text_current = "";

    //Load model to shader _program_ptr
	LoadModel();

}
void rmText2D::LoadModel(){




}
void rmText2D::Update(float dt){
    // Update the data (buffer variables) here
}
void rmText2D::Update(ROS_INTERFACE &ros_interface){
    // Update the data (buffer variables) here
}
void rmText2D::Update(ROS_API &ros_api){
    // Update the data (buffer variables) here
}
void rmText2D::Render(std::shared_ptr<ViewManager> _camera_ptr){
    // test
    static int _count = 0;
    //
    glUseProgram(0); // Program 0: OpenGL ver1.0
    selectFont2D(2);
    text2D_output(0.5,0.5, "This is 2D text: " + std::to_string(_count++) );
}



//
void rmText2D::selectFont2D(int newfont){
  font2D = fonts2D[newfont];
}
void rmText2D::text2D_output(float x, float y, std::string string_in){
  glRasterPos2f(x, y);
  for (size_t i = 0; i < string_in.size(); i++) {
    glutBitmapCharacter(font2D, string_in[i]);
  }
}
