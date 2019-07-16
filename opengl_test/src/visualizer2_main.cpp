#include <ROS_ICLU3_v0.hpp>
#include "../external/AntTweakBar-1.16/include/AntTweakBar.h"
#include <setjmp.h> // For leaving main loop
//
#include "Common.h"
// #include "ViewManager.h"
#include "ViewManager_v2.h"
#include "Scene.h"
#include "SCENE_W_main.h"
#include "SCENE_W0.h"
#include "SCENE_W1.h"
#include "SCENE_W2.h"
#include "SCENE_W3.h"
#include "SCENE_W4.h"
#include "SCENE_W5.h"
#include "SCENE_W6.h"

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
// #define __OPENCV_WINDOW__


// The following is for point-sprite
#define NUM_POINTCLOUT_MAX 1000000

// ROS_interface for ICLU3, ver.0
ROS_API ros_api;
// The scene for rendering
std::vector< std::shared_ptr<Scene> > all_scenes;
// std::shared_ptr<Scene> scene_ptr;




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
float	windows_width = 1200; // 800;
float   windows_height = 800; // 600;
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







// AntTwekBar
//----------------------------------------------------//
// Shape			m_shape;
ViewManager		m_camera;
TwBar			*bar_1_ptr;
vec2			m_screenSize;
// vector<Shape>   m_shapes;
int				m_currentCamera;
int             m_currentCamera_old;
float			m_zoom = 3.0f;
// float			m_fps_d = 0;
std::string     m_fps_d_str = "0.0";
// std::vector<float> m_fps_topic;
std::vector<std::string> m_fps_topic_str;
unsigned int	m_frames = 0;
unsigned int    m_currentTime = 0;
unsigned int    m_timebase = 0;
bool			m_autoRotate;
bool			m_isOthogonol;
vec3			m_backgroundColor = vec3(0.486, 0.721, 0.918);
//
typedef enum { CAM_FOLLOW = 0, CAM_STATIC, NUM_CAMERA_MODE } ModelCamera;

// void TW_CALL SetAutoRotateCB(const void *value, void *clientData)
// {
// 	// m_autoRotate = *(const int *)value;
// }
// void TW_CALL GetAutoRotateCB(void *value, void *clientData)
// {
// 	// *(int *)value = m_autoRotate;
// }
void TW_CALL SetStaticCam(const void *value, void *clientData)
{
	// m_isOthogonol = *(const int *)value;
	// m_camera.ToggleOrtho();
    for (size_t i=0; i < all_scenes.size(); ++i){
        // all_scenes[i]->KeyBoardEvent('c', ros_api);
        all_scenes[i]->switchCameraMotionMode( *(int *)(value), ros_api);
    }
}
void TW_CALL GetStaticCam(void *value, void *clientData)
{
	// *(int *)value = m_isOthogonol;
    *(int *)value = all_scenes[0]->get_camera_motion_mode();
}
void TW_CALL ResetViewCB(void * clientData)
{
    /*
	m_camera.SetRotation(0, 0);
	for (int i = 0; i < m_shapes.size(); ++i)
	{
		m_shapes[i].rotation = vec3(0);
	}
    */

    for (size_t i=0; i < all_scenes.size(); ++i){
        all_scenes[i]->KeyBoardEvent('z', ros_api);
    }
	glutPostRedisplay();
}

void setupGUI()
{
	// Initialize AntTweakBar
	//TwDefine(" GLOBAL fontscaling=2 ");

#ifdef _MSC_VER
	TwInit(TW_OPENGL, NULL);
#elif  __GNUC__ // Compiler for cross platform app., including Linux
    TwInit(TW_OPENGL, NULL);
#else
	TwInit(TW_OPENGL_CORE, NULL);
#endif

	TwGLUTModifiersFunc(glutGetModifiers); // <-- This is just for key modifiers


    bar_1_ptr = TwNewBar("Properties");
    TwDefine(" Properties position='0 0' ");
	TwDefine(" Properties size='270 450' "); // 220, 300
	TwDefine(" Properties fontsize='3' color='0 0 0' alpha=180 ");  // http://anttweakbar.sourceforge.net/doc/tools:anttweakbar:twbarparamsyntax

    // FPS of display
    // TwAddVarRO(bar_1_ptr, "fps_d", TW_TYPE_FLOAT, &m_fps_d, " label='FPS-display' help='Frame Per Second(FPS)' ");
	TwAddVarRO(bar_1_ptr, "fps_d", TW_TYPE_STDSTRING, &m_fps_d_str, " label='FPS-display' help='Frame Per Second(FPS)' ");
    //
    TwAddSeparator(bar_1_ptr, "Sep1", "");
    // FPS of camera
    // m_fps_topic.resize( ros_api.ros_interface.get_count_of_all_topics(), 0.0f);
    // for (int topic_idx = int(MSG_ID::camera_front_right); topic_idx < ros_api.ros_interface.get_count_of_all_topics(); ++topic_idx ){
    //     TwAddVarRO(bar_1_ptr, ("fps_" + std::to_string(topic_idx)).c_str(), TW_TYPE_FLOAT, &(m_fps_topic[topic_idx]), (" label='FPS-" + ros_api.ros_interface.get_topic_name(topic_idx) + "' help='Frame Per Second(FPS)' ").c_str() );
    // }
    m_fps_topic_str.resize( ros_api.ros_interface.get_count_of_all_topics(), "0.0");
    for (int topic_idx = int(MSG_ID::camera_front_right); topic_idx < ros_api.ros_interface.get_count_of_all_topics(); ++topic_idx ){
        TwAddVarRO(bar_1_ptr, ("fps_" + std::to_string(topic_idx)).c_str(), TW_TYPE_STDSTRING, &(m_fps_topic_str[topic_idx]), (" label='FPS-" + ros_api.ros_interface.get_topic_name(topic_idx) + "' help='Frame Per Second(FPS)' ").c_str() );
    }
    //
    TwAddSeparator(bar_1_ptr, "Sep2", "");
    // menu
	{
		TwEnumVal cameraEV[NUM_CAMERA_MODE] = { { CAM_FOLLOW, "follow" },{ CAM_STATIC, "static" } };
		TwType cameraType = TwDefineEnum("camType", cameraEV, NUM_CAMERA_MODE);
		TwAddVarRW(bar_1_ptr, "CamMode", cameraType, &m_currentCamera, " keyIncr='<' keyDecr='>' help='Change view mode.' ");
	}

	// TwAddVarRW(bar_1_ptr, "Zoom", TW_TYPE_FLOAT, &m_zoom, " min=0.01 max=3.0 step=0.01 help='Camera zoom in/out' ");
	// TwAddVarRW(bar_1_ptr, "BackgroundColor", TW_TYPE_COLOR3F, value_ptr(m_backgroundColor), " label='Background Color' opened=true help='Used in glClearColor' ");
	// TwAddVarCB(bar_1_ptr, "AutoRotate", TW_TYPE_BOOL32, SetAutoRotateCB, GetAutoRotateCB, NULL, " label='Auto-rotate' key=space help='Toggle auto-rotate mode.' ");

    TwAddSeparator(bar_1_ptr, "Sep3", "");
    TwAddVarCB(bar_1_ptr, "IsStaticCam", TW_TYPE_BOOL32, SetStaticCam, GetStaticCam, NULL, " label='Is static view' key=space help='Is static view mode' ");
	TwAddButton(bar_1_ptr, "ResetView", ResetViewCB, NULL, " label='Reset view' ");
}
//----------------------------------------------------//







// OpenGL, GLUT
//----------------------------------------------------//
void My_Init()
{
    // glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	// glEnable(GL_DEPTH_TEST);
	// glDepthFunc(GL_LEQUAL);

    // scene_ptr.reset(new Scene(ros_api.get_pkg_path()) );
    all_scenes.push_back( std::shared_ptr<Scene>( new SCENE_W_main(ros_api.get_pkg_path()) ) );
    all_scenes.push_back( std::shared_ptr<Scene>( new SCENE_W0(ros_api.get_pkg_path()) ) );
    all_scenes.push_back( std::shared_ptr<Scene>( new SCENE_W1(ros_api.get_pkg_path()) ) );
    all_scenes.push_back( std::shared_ptr<Scene>( new SCENE_W2(ros_api.get_pkg_path()) ) );
    all_scenes.push_back( std::shared_ptr<Scene>( new SCENE_W3(ros_api.get_pkg_path()) ) );
    all_scenes.push_back( std::shared_ptr<Scene>( new SCENE_W4(ros_api.get_pkg_path()) ) );
    all_scenes.push_back( std::shared_ptr<Scene>( new SCENE_W5(ros_api.get_pkg_path()) ) );
    all_scenes.push_back( std::shared_ptr<Scene>( new SCENE_W6(ros_api.get_pkg_path()) ) );

    // Clear background color
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
	glDepthFunc(GL_LEQUAL);
	glDepthRange(0.0f, 1.0f);
}

TIME_STAMP::Period period_frame_pre("pre frame");
TIME_STAMP::Period period_frame_post("post frame");
TIME_STAMP::FPS    fps_display("fps_display");
void My_Display()
{
    // FPS of the display
    fps_display.stamp();
    // m_fps_d = fps_display.fps;
    m_fps_d_str = all_header::to_string_p(fps_display.fps, 1);
    // FPS for topics
    // for (size_t i=0; i < m_fps_topic.size(); ++i){
    //     m_fps_topic[i] = ros_api.fps_list[i].fps;
    // }
    for (size_t i=0; i < m_fps_topic_str.size(); ++i){
        m_fps_topic_str[i] = all_header::to_string_p(ros_api.fps_list[i].fps, 1);
    }

    // m_currentTime = glutGet(GLUT_ELAPSED_TIME);
	// if (m_currentTime - m_timebase > 1000)
	// {
	// 	m_fps_d = (m_frames * 1000) / (m_currentTime - m_timebase);
	// 	m_frames = 0;
	// 	m_timebase = m_currentTime;
    //     // std::cout << "m_fps_d = " << m_fps_d << "\n";
	// }
	// m_frames++;

    // Camera mode
    if (m_currentCamera != m_currentCamera_old){
        for (size_t i=0; i < all_scenes.size(); ++i){
            // all_scenes[i]->KeyBoardEvent('c', ros_api);
            all_scenes[i]->switchCameraMotionMode( m_currentCamera, ros_api);
        }
        m_currentCamera = all_scenes[0]->get_camera_motion_mode();
        m_currentCamera_old = m_currentCamera;
    }else{
        m_currentCamera = all_scenes[0]->get_camera_motion_mode();
    }
    TwRefreshBar(bar_1_ptr);


    // evaluation
    TIME_STAMP::Period period_in("part");
    TIME_STAMP::Period period_all_func("full display");
    //
    // period_frame_pre.stamp();   period_frame_pre.show_msec();   period_frame_pre.show_jitter_usec();
    //
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


    // Update the "_latest_tf_common_update_time"
    // ros_interface.update_latest_tf_common_update_time("map", "base");
    ros_api.ros_interface.set_global_delay(0.2);
    ros_api.ros_interface.update_current_slice_time();
    // ros_api.ros_interface.set_ref_frame("base"); <-- do this in Scene (base class with camera mode selection)

    // Update data
    bool is_updated = ros_api.update();

    /*
    // FPS show
    for (size_t i=0; i < ros_api.fps_list.size(); ++i){
        if (ros_api.got_on_any_topic[i])
            ros_api.fps_list[i].show();
    }
    // end FPS show
    */


#ifdef __DEBUG__
    // evaluation
    // period_in.stamp();  period_in.show_msec();
    //
#endif

    // Update all_scenes
    //--------------------//
    for (size_t i=0; i < all_scenes.size(); ++i){
        all_scenes[i]->Update(ros_api);
        // all_scenes[i]->Update(ros_api.ros_interface);
    }
    //--------------------//

    // Interacting with ROS topic (on behave of each scene)
    //--------------------//
    for (size_t i=0; i < all_scenes.size(); ++i){
        all_scenes[i]->ROSTopicEvent(ros_api);
    }
    //--------------------//


#ifdef __DEBUG__
    // evaluation
    // period_in.stamp();   period_in.show_msec();
    //
#endif
    /*
    // test, showing speed
    std::shared_ptr< msgs::VehInfo > _veh_info_ptr;
    if (ros_api.get_message( int(MSG_ID::vehicle_info_1), _veh_info_ptr)){
        std::cout << "Speed (km/h): " << (_veh_info_ptr->ego_speed)*3.6 << ", ";
        std::cout << "yaw_rate (deg/s): " << (_veh_info_ptr->yaw_rate) << "\n";
    }
    */
    /*
    // test, showing operations
    std::shared_ptr< opengl_test::GUI2_op > _GUI2_op_ptr;
    if (ros_api.get_message( int(MSG_ID::GUI_operatio), _GUI2_op_ptr)){
        std::cout << "---\n";
        std::cout << "cam_type: " << _GUI2_op_ptr->cam_type << "\n";
        std::cout << "image3D: " << _GUI2_op_ptr->image3D << "\n";
        std::cout << "image_surr: " << _GUI2_op_ptr->image_surr << "\n";
        std::cout << "cam_motion: " << _GUI2_op_ptr->cam_motion << "\n";
        //
        all_scenes[0]->KeyBoardEvent('c', ros_api);
        //
        ros_api.ros_interface.send_GUI2_op( int(MSG_ID::GUI_operatio), *_GUI2_op_ptr);
    }
    //
    */

    //---------------------------------//
    // end ROS_interface


#ifdef __OPENCV_WINDOW__
    // Image
    int num_image = NUM_IMAGE;
    int image_topic_id = int(MSG_ID::camera_front_right);
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

    // Note: The following operations are move into the render function of each Scene,
    //       which means that each Scene will have their saparated window and we should not draw two Scene into one window
    // glViewport(0, 0, windows_width, windows_height); // <-- move to Draw()
    // glViewport(100, 100, windows_width/2, windows_height/2); // <-- move to Draw()
    // glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    // glClearDepth(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render all_scenes
    //--------------------//
    for (size_t i=0; i < all_scenes.size(); ++i){
        all_scenes[i]->Render();
    }
    //--------------------//

    // Render AntTweeekBar
    TwDraw();

    //--------------------//
    glutSwapBuffers();
    //---------------------------------//


    //=============================================================//
    // end Evaluation

#ifdef __DEBUG__
    // evaluation
    // period_in.stamp();  period_in.show_msec();
    // period_all_func.stamp();    period_all_func.show_msec();
    // period_frame_post.stamp();  period_frame_post.show_msec();  period_frame_post.show_jitter_usec();
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

    // Render all_scenes
    //--------------------//
    for (size_t i=0; i < all_scenes.size(); ++i){
        // all_scenes[i]->GetCamera()->SetWindowSize(windows_width, windows_height);
        all_scenes[i]->Reshape(windows_width, windows_height);
    }
    //--------------------//

    // AntTweakBar
    TwWindowSize(width, height);
}

//Timer event
void My_Timer(int val)
{
    glutTimerFunc(timer_interval, My_Timer, val);
    // test
    // std::cout << "in My_Timer()\n";
    // std::this_thread::sleep_for( std::chrono::milliseconds(100) );

    // Update all_scenes
    //--------------------//
    for (size_t i=0; i < all_scenes.size(); ++i){
        all_scenes[i]->Update(timer_interval);
    }
    //--------------------//

	glutPostRedisplay();
	// glutTimerFunc(timer_interval, My_Timer, val);
}

//Mouse event
void My_Mouse(int button, int state, int x, int y)
{
    if (TwEventMouseButtonGLUT(button, state, x, y)){
        TwRefreshBar(bar_1_ptr);
        return;
    }
    // Update all_scenes
    //--------------------//
    for (size_t i=0; i < all_scenes.size(); ++i){
        all_scenes[i]->MouseEvent(button, state, x, y);
    }
    //--------------------//

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
    if (TwEventKeyboardGLUT(key, x, y)){
        return;
    }

    // Update all_scenes
    //--------------------//
    for (size_t i=0; i < all_scenes.size(); ++i){
        all_scenes[i]->KeyBoardEvent(key, ros_api);
    }
    //--------------------//
}

//Special key event
void My_SpecialKeys(int key, int x, int y)
{
    if (TwEventSpecialGLUT(key, x, y)){
        return;
    }
    // Update all_scenes
    //--------------------//
    for (size_t i=0; i < all_scenes.size(); ++i){
        all_scenes[i]->KeyBoardEvent(key);
    }
    //--------------------//
}

/*
//Menu event
void My_Menu(int id)
{
	scene_ptr_1->MenuEvent(id);

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
    if (TwEventMouseMotionGLUT(x, y)){
        return;
    }
    // Update all_scenes
    //--------------------//
    for (size_t i=0; i < all_scenes.size(); ++i){
        all_scenes[i]->GetCamera()->mouseMoveEvent(x, y);
    }
    //--------------------//
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

    // AntTweakBar
    setupGUI();

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
