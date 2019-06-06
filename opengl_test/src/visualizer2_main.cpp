#include <ROS_interface.hpp>
#include <setjmp.h> // For leaving main loop
//
#include "Common.h"
#include "ViewManager.h"
#include "Scene.h"

// Debug
#include <iostream>
// test
#include <FILTER_LIB.h>



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


// #define M_PI = 3.14;
using std::vector;
using std::string;
using namespace cv;
//
using namespace glm;
using namespace std;

/*
#define MENU_Sale 1
#define MENU_Shrink 2
#define MENU_EXIT   3
*/

// The scene for rendering
//------------------------------//
Scene *scene_ptr;
//------------------------------//







#define NUM_IMAGE 9
#define NUM_POINTCLOUT_MAX 1000000

#define __DEBUG__
#define __OPENCV_WINDOW__
#define __SUB_POINT_CLOUD__




// float	aspect;
float	windows_init_width = 600;
float   windows_init_height = 600;
float	timer_interval = 16.0f;



// ROS things (through ROS_interface)
//----------------------------------------//
ROS_INTERFACE ros_interface;
std::string path_pkg_directory("/home/benson516_itri/catkin_ws/src/opengl_test_ROS/opengl_test/");
// nickname for topic_id
enum class MSG_ID{
    camera_0,
    camera_1,
    camera_2,
    camera_3,
    camera_4,
    camera_5,
    camera_6,
    camera_7,
    camera_8,
    //
    point_cloud_1
};
// Declare outside the loop to avoid periodically construction and destruction.
vector< std::shared_ptr< cv::Mat > > image_out_ptr_list(9);
std::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > pc_out_ptr;
#ifdef __OPENCV_WINDOW__
    vector<string> window_names;
#endif
//----------------------------------------//


//----------------------------------------------------//
void ROS_init(int argc, char *argv[]){
    // Setup the ROS interface
    ros_interface.setup_node(argc, argv, "visualizer2");


    {
        using MSG::M_TYPE;
        // Image
        ros_interface.add_a_topic("/camera/1/0/image_sync", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/1/1/image_sync", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/1/2/image_sync", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/0/2/image_sync", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/2/0/image", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/2/1/image", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/0/0/image", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/0/1/image", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/2/2/image", int(M_TYPE::Image), true, 1, 3);
        // ITRIPointCloud
#ifdef __SUB_POINT_CLOUD__
        ros_interface.add_a_topic("LidFrontLeft_sync", int(M_TYPE::ITRIPointCloud), true, 5, 5);
#endif
    }
    //------------------------------------------------//

    // start
    ros_interface.start();


    // Showing Image by cv show
    int num_image = NUM_IMAGE;
#ifdef __OPENCV_WINDOW__
    // OpenCV windows
    // vector<string> window_names;
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
	scene_ptr = new Scene(path_pkg_directory);
}

// GLUT callback. Called to draw the scene.
size_t num_points = NUM_POINTCLOUT_MAX;
auto start_old = std::chrono::high_resolution_clock::now();
auto t_image_old_0 = std::chrono::high_resolution_clock::now();
auto t_pc_old = std::chrono::high_resolution_clock::now();
long long image_period_us_0 = 1000000;
long long pc_period_us = 1000000;
LPF image_period_lpf_0(1/60.0, 1.0);
LPF pc_period_lpf_0(1/60.0, 1.0);
//
void My_Display()
{
    auto start = std::chrono::high_resolution_clock::now();
    // Evaluation
    //=============================================================//

    // ROS_interface
    //---------------------------------//
    // Check if it's time to leave the main loop
    if (!ros_interface.is_running()){
        std::cout << "Leaving main loop\n";
        leave_main_loop();
        std::cout << "Quit main loop\n";
    }


    // Image
    int num_image = NUM_IMAGE;
    int image_topic_id = int(MSG_ID::camera_0);
    //
    // vector<cv::Mat> image_out_list(num_image);
    vector<bool> is_image_updated(num_image, false);
    for (size_t i=0; i < num_image; ++i){
        is_image_updated[i] = ros_interface.get_Image( (image_topic_id+i), image_out_ptr_list[i]);
    }
    // std::cout << "Drawing images\n";


#ifdef __OPENCV_WINDOW__
    for (size_t i=0; i < num_image; ++i){
        if (is_image_updated[i]){
            if (i == 0){
                // test, time
                //--------------------------//
                auto t_image_new_0 = std::chrono::high_resolution_clock::now();
                auto image_period_0 = t_image_new_0 - t_image_old_0;
                t_image_old_0 = t_image_new_0;
                image_period_us_0 = std::chrono::duration_cast<std::chrono::microseconds>(image_period_0).count();
                image_period_lpf_0.filter(image_period_us_0);
                //--------------------------//
            }
            // std::cout << "Drawing an image\n";
            imshow(window_names[i], *image_out_ptr_list[i]);
            // std::cout << "got one image\n";
            waitKey(1);
        }
    }
#endif




#ifdef __SUB_POINT_CLOUD__
    // ITRIPointCloud
    int num_pointcloud = 1;
    // ITRIPointCloud
    int ITRIPointCloud_topic_id = int(MSG_ID::point_cloud_1);
    // pcl::PointCloud<pcl::PointXYZI> pc_out;
    bool pc_result = ros_interface.get_ITRIPointCloud( (ITRIPointCloud_topic_id), pc_out_ptr);
    if (pc_result){
        // test, time
        //--------------------------//
        auto t_pc_new = std::chrono::high_resolution_clock::now();
        auto pc_period = t_pc_new - t_pc_old;
        t_pc_old = t_pc_new;
        pc_period_us = std::chrono::duration_cast<std::chrono::microseconds>(pc_period).count();
        pc_period_lpf_0.filter(pc_period_us);
        //--------------------------//
    }
#endif
    //---------------------------------//
    // end ROS_interface


    // OpenGL, GLUT
    //---------------------------------//
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
    std::cout << "execution time (ms): " << elapsed_us*0.001 << ",\t";
    std::cout << "Image[0] rate: " << (1000000.0/image_period_lpf_0.output) << " fps\t";
    std::cout << "PointCloud rate: " << (1000000.0/pc_period_lpf_0.output) << " fps\t";
    std::cout << "\n";
#endif
}

//Call to resize the window
void My_Reshape(int width, int height)
{
	scene_ptr->GetCamera()->SetWindowSize(width, height);
	glViewport(0, 0, width, height);
}

//Timer event
void My_Timer(int val)
{
	scene_ptr->Update(timer_interval);
	glutPostRedisplay();
	glutTimerFunc(timer_interval, My_Timer, val);
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
	glutInitWindowSize(windows_init_width, windows_init_height);
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


    // ROS_interface
    ROS_init(argc, argv);

	//進入主迴圈
	// glutMainLoop();
    enter_main_loop();

    std::cout << "Leaving progeam.\n";
	return 0;
}
