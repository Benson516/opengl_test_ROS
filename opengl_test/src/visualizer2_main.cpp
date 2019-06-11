#include <ROS_ICLU3_v0.hpp>
#include <setjmp.h> // For leaving main loop
//
#include "Common.h"
#include "ViewManager.h"
#include "Scene.h"

// Debug
#include <iostream>
// test
#include <FILTER_LIB.h>

// #define M_PI = 3.14;
using std::vector;
using std::string;
using namespace cv;
//
using namespace glm;
using namespace std;

//
#define __DEBUG__
#define __OPENCV_WINDOW__


// The following is for point-sprite
#define NUM_POINTCLOUT_MAX 1000000

// ROS_interface for ICLU3, ver.0
ROS_ICLU3_V0 ros_api;
// The scene for rendering
std::shared_ptr<Scene> scene_ptr;



// For leaving the main loop
//--------------------------------//
static jmp_buf jmpbuf;
static bool jmp_set = false;
void enter_main_loop () {
    if (!setjmp(jmpbuf)) {
        jmp_set = true;
        glutMainLoop();
    }
    jmp_set = false;
}
void leave_main_loop () {
    if (jmp_set) longjmp(jmpbuf, 1);
}
//--------------------------------//





// float	aspect;
float	windows_width = 800;
float   windows_height = 600;
float	timer_interval = 16.0f;
/*
#define MENU_Sale 1
#define MENU_Shrink 2
#define MENU_EXIT   3
*/


// Image, cv windoes
//---------------------------------------------------//
#ifdef __OPENCV_WINDOW__
    #define NUM_IMAGE 9
    vector<string> window_names;
    // Declare outside the loop to avoid periodically construction and destruction.
    vector< std::shared_ptr< cv::Mat > > image_out_ptr_list(9);
#endif
void cv_windows_setup(){
    // Showing Image by cv show
#ifdef __OPENCV_WINDOW__
    int num_image = NUM_IMAGE;
    // OpenCV windows
    for (size_t i=0; i < num_image; ++i){
        std::stringstream _ss_window_name;
        _ss_window_name << "image_" << i;
        namedWindow(_ss_window_name.str(), cv::WINDOW_AUTOSIZE);
        window_names.push_back( _ss_window_name.str() );
    }
#endif
}
//----------------------------------------------------//



// OpenGL, GLUT
//----------------------------------------------------//
void My_Init()
{
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	scene_ptr.reset(new Scene(ros_api.get_pkg_path()) );
}

// GLUT callback. Called to draw the scene.
auto start_old = std::chrono::high_resolution_clock::now();
//
void My_Display()
{
    // std::cout << "in My_Display()\n";
    auto start = std::chrono::high_resolution_clock::now();
    // Evaluation
    //=============================================================//



    // ROS_interface
    //---------------------------------//
    // Check if it's time to leave the main loop
    if (!ros_api.is_running()){
        std::cout << "Leaving main loop\n";
        leave_main_loop();
        // exit(0);
    }
    // Update data
    // bool is_updated = ros_api.update();
    scene_ptr->Update(ros_api.ros_interface);
    //---------------------------------//
    // end ROS_interface


    #ifdef __OPENCV_WINDOW__
        // Image
        int num_image = NUM_IMAGE;
        int image_topic_id = int(MSG_ID::camera_0);
        vector<bool> is_image_updated(num_image, false);
        for (size_t i=0; i < num_image; ++i){
            is_image_updated[i] = ros_api.ros_interface.get_Image( (image_topic_id+i), image_out_ptr_list[i]);
        }
        for (size_t i=0; i < num_image; ++i){
            if (is_image_updated[i]){
                imshow(window_names[i], *image_out_ptr_list[i]);
                waitKey(1);
            }
        }
    #endif

    // OpenGL, GLUT
    //---------------------------------//
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glViewport(0, 0, windows_width, windows_height); // <-- move to Draw()
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	scene_ptr->Render();

    glutSwapBuffers();
    //---------------------------------//


    //=============================================================//
    // end Evaluation
#ifdef __DEBUG__
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    auto period = start - start_old;
    start_old = start;
    long long elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    long long period_us = std::chrono::duration_cast<std::chrono::microseconds>(period).count();
    // std::cout << "execution time (ms): " << elapsed_us*0.001 << ",\t";
    // std::cout << "loop period (ms): " << period_us*0.001;
    // std::cout << "---\n";
#endif
}
// end My_Display()

//Call to resize the window
void My_Reshape(int width, int height)
{
    windows_width = width;
    windows_height = height;
    // glViewport(0, 0, windows_width, windows_height); // <-- move to Draw()
	scene_ptr->GetCamera()->SetWindowSize(windows_width, windows_height);
}

//Timer event
void My_Timer(int val)
{
    glutTimerFunc(timer_interval, My_Timer, val);
    // test
    // std::cout << "in My_Timer()\n";
    // std::this_thread::sleep_for( std::chrono::milliseconds(100) );
    //
    scene_ptr->Update(timer_interval);
	glutPostRedisplay();
	// glutTimerFunc(timer_interval, My_Timer, val);
}

//Mouse event
void My_Mouse(int button, int state, int x, int y)
{
	scene_ptr->MouseEvent(button, state, x, y);

    /*
	if (button == GLUT_LEFT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			printf("Mouse %d is pressed at (%d, %d)\n", button, x, y);
		}
		else if (state == GLUT_UP)
		{
			printf("Mouse %d is released at (%d, %d)\n", button, x, y);
		}
	}
	else if (button == GLUT_RIGHT_BUTTON)
	{
		printf("Mouse %d is pressed\n", button);
	}
    */
}

//Keyboard event
void My_Keyboard(unsigned char key, int x, int y)
{
	scene_ptr->KeyBoardEvent(key);
}

//Special key event
void My_SpecialKeys(int key, int x, int y)
{
	scene_ptr->KeyBoardEvent(key);
}

/*
//Menu event
void My_Menu(int id)
{
	scene_ptr->MenuEvent(id);

	switch(id)
	{
	case MENU_EXIT:
		exit(0);
		break;
	default:
		break;
	}
}
*/



void My_Mouse_Moving(int x, int y) {
	scene_ptr->GetCamera()->mouseMoveEvent(x, y);
}

int main(int argc, char *argv[])
{

    // ROS_interface
    ros_api.start(argc, argv, "visualizer2");



#ifdef __APPLE__
    //Change working directory to source code path
    chdir(__FILEPATH__("/../Assets/"));
#endif
	// Initialize GLUT and GLEW, then create a window.
	////////////////////
	glutInit(&argc, argv);


#ifdef _MSC_VER // Compiler for VisualStudio
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
#elif __GNUC__ // Compiler for cross platform app., including Linux
    // glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitDisplayMode(GLUT_RGBA | GLUT_ALPHA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_MULTISAMPLE);
#else
    glutInitDisplayMode(GLUT_3_2_CORE_PROFILE | GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
#endif

	glutInitWindowPosition(100, 100);
	glutInitWindowSize(windows_width, windows_height);
	glutCreateWindow("Visualizer2"); // You cannot use OpenGL functions before this line; The OpenGL context must be created first by glutCreateWindow()!
#ifdef _MSC_VER // Compiler for VisualStudio
	glewInit();
#elif  __GNUC__ // Compiler for cross platform app., including Linux
    glewInit();
#endif

	//Print debug information
	Common::DumpInfo();

	//Call custom initialize function
	My_Init();

    /*
	//定義選單
	////////////////////
	int menu_main = glutCreateMenu(My_Menu);
	int menu_entry = glutCreateMenu(My_Menu);

	glutSetMenu(menu_main);
	glutAddSubMenu("Scale", menu_entry);
	glutAddMenuEntry("Exit", MENU_EXIT);

	glutSetMenu(menu_entry);
	glutAddMenuEntry("*2.0", MENU_Sale);
	glutAddMenuEntry("*0.5", MENU_Shrink);

	glutSetMenu(menu_main);
	// glutAttachMenu(GLUT_RIGHT_BUTTON);
    // glutAttachMenu(GLUT_MIDDLE_BUTTON);
	////////////////////
    */

	//Register GLUT callback functions
	////////////////////
	glutDisplayFunc(My_Display);
	glutReshapeFunc(My_Reshape);
	glutMouseFunc(My_Mouse);
	glutKeyboardFunc(My_Keyboard);
	glutSpecialFunc(My_SpecialKeys);
	glutTimerFunc(timer_interval, My_Timer, 0);
	glutPassiveMotionFunc(My_Mouse_Moving);
	glutMotionFunc(My_Mouse_Moving);
	////////////////////




    // test, cv windows
    cv_windows_setup();

	//進入主迴圈
	// glutMainLoop();
    enter_main_loop();

    std::cout << "Leaving progeam.\n";
	return 0;
}
