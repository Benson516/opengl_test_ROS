#include <ROS_interface.hpp>
//
#include "Common.h"
#include <iostream>
#include <string>
//
#include <setjmp.h> // For leaving main loop

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


#define STRING_TOPIC_COUNT 6
#define NUM_IMAGE 9

#define __OPENCV_WINDOW__
#define __SUB_POINT_CLOUD__




#define M_PI = 3.14;

using std::vector;
using std::string;
using namespace cv;
//
using namespace glm;
using namespace std;






ROS_INTERFACE* ros_interface_ptr=NULL;
// nickname for topic_id
enum class MSG_ID{
    chatter_0,
    chatter_1,
    chatter_2,
    chatter_3,
    chatter_4,
    chatter_5,

    camera_0,
    camera_1,
    camera_2,
    camera_3,
    camera_4,
    camera_5,
    camera_6,
    camera_7,
    camera_8,

    point_cloud_1
};
vector<string> window_names;





//uniform id
struct
{
	GLint  mv_matrix;
	GLint  proj_matrix;
} uniforms;


GLuint			program;			//shader program
mat4			proj_matrix;		//projection matrix
GLuint          vao;
GLuint			vertex_shader;
GLuint			fragment_shader;
GLuint          buffer;
GLint           mv_location;
GLint           proj_location;
GLint			time_Loc;

GLuint m_texture;
static unsigned int seed = 0x13371337;

static inline float random_float()
{
	float res;
	unsigned int tmp;

	seed *= 16807;

	tmp = seed ^ (seed >> 4) ^ (seed << 15);

	*((unsigned int *)&res) = (tmp >> 9) | 0x3F800000;

	return (res - 1.0f);
}

enum
{
	NUM_STARS = 1000000
};

void My_Init()
{
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glEnable(GL_DEPTH_TEST);

    std::cout << "here\n";

	//Initialize shaders
	///////////////////////////
	program = glCreateProgram();
	std::cout << "here\n";

	GLuint vs = glCreateShader(GL_VERTEX_SHADER);
	GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    //
    std::string path_pkg_directory("/home/benson516_itri/catkin_ws/src/opengl_test_ROS/opengl_test/");
    std::string path_vs("Assets/7.5.Point_Sprite/Point_Sprite.vs.glsl");
    std::string path_fs("Assets/7.5.Point_Sprite/Point_Sprite.fs.glsl");
    path_vs = path_pkg_directory + path_vs;
    path_fs = path_pkg_directory + path_fs;
	char** vsSource = LoadShaderSource(path_vs.c_str());
	char** fsSource = LoadShaderSource(path_fs.c_str());

    std::cout << "here\n";

	glShaderSource(vs, 1, vsSource, NULL);
	glShaderSource(fs, 1, fsSource, NULL);
	FreeShaderSource(vsSource);
	FreeShaderSource(fsSource);
	glCompileShader(vs);
	glCompileShader(fs);
	ShaderLog(vs);
	ShaderLog(fs);

	//Attach Shader to program
	glAttachShader(program, vs);
	glAttachShader(program, fs);
	glLinkProgram(program);

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	proj_location = glGetUniformLocation(program, "proj_matrix");
	time_Loc = glGetUniformLocation(program, "time");

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	struct star_t
	{
		glm::vec3     position;
		glm::vec3     color;
	};

	glGenBuffers(1, &buffer);
	glBindBuffer(GL_ARRAY_BUFFER, buffer);
	glBufferData(GL_ARRAY_BUFFER, NUM_STARS * sizeof(star_t), NULL, GL_STATIC_DRAW);
	// glBufferData(GL_ARRAY_BUFFER, NUM_STARS * sizeof(star_t), NULL, GL_DYNAMIC_DRAW);

	star_t * star = (star_t *)glMapBufferRange(GL_ARRAY_BUFFER, 0, NUM_STARS * sizeof(star_t), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
	int i;

	for (i = 0; i < NUM_STARS; i++)
	{
		star[i].position[0] = (random_float() * 2.0f - 1.0f) * 100.0f;
		star[i].position[1] = (random_float() * 2.0f - 1.0f) * 100.0f;
		star[i].position[2] = random_float();
		star[i].color[0] = 0.8f + random_float() * 0.2f;
		star[i].color[1] = 0.8f + random_float() * 0.2f;
		star[i].color[2] = 0.8f + random_float() * 0.2f;
	}

	glUnmapBuffer(GL_ARRAY_BUFFER);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(star_t), NULL);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(star_t), (void *)sizeof(glm::vec3));
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnable(GL_TEXTURE_2D);
	glActiveTexture(GL_TEXTURE0);


    std::string path_tdata("Assets/7.5.Point_Sprite/star.png");
    path_tdata = path_pkg_directory + path_tdata;
	TextureData tdata = Load_png(path_tdata.c_str());

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glGenTextures(1, &m_texture);
	glBindTexture(GL_TEXTURE_2D, m_texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tdata.width, tdata.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, tdata.data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);;
}

// GLUT callback. Called to draw the scene.
void My_Idle(){
    // Update screen later
    glutPostRedisplay();
}
// GLUT callback. Called to draw the scene.
void My_Display()
{


    ROS_INTERFACE &ros_interface = (*ros_interface_ptr);
    //
    if (!ros_interface.is_running()){
        std::cout << "Leaving main loop\n";
        leave_main_loop();
        std::cout << "Quit main loop\n";
    }
    int num_image = NUM_IMAGE;
    int image_topic_id = int(MSG_ID::camera_0);
    //

    vector<cv::Mat> image_out_list(num_image);
    vector<bool> is_image_updated(num_image, false);
    for (size_t i=0; i < num_image; ++i){
        is_image_updated[i] = ros_interface.get_Image( (image_topic_id+i), image_out_list[i]);
    }
    // std::cout << "Drawing images\n";
#ifdef __OPENCV_WINDOW__
    for (size_t i=0; i < num_image; ++i){
        if (is_image_updated[i]){
            // std::cout << "Drawing an image\n";
            imshow(window_names[i], image_out_list[i]);
            // std::cout << "got one image\n";
            waitKey(1);
        }
    }
#endif




	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Update shaders' input variable
	///////////////////////////
	static const GLfloat black[] = { 0.0f, 0, 0.0f, 1.0f };
	static const GLfloat one = 1.0f;

	glClearBufferfv(GL_COLOR, 0, black);
	glClearBufferfv(GL_DEPTH, 0, &one);

	glUseProgram(program);

	float f_timer_cnt = glutGet(GLUT_ELAPSED_TIME);
	float currentTime = f_timer_cnt* 0.001f;

	currentTime *= 0.1f;
	currentTime -= floor(currentTime);

	glUniform1f(time_Loc, currentTime);
	glUniformMatrix4fv(proj_location, 1, GL_FALSE, &proj_matrix[0][0]);

	glEnable(GL_POINT_SPRITE);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, m_texture);
	glEnable(GL_PROGRAM_POINT_SIZE);
 	glDrawArrays(GL_POINTS, 0, NUM_STARS);
	/*
	// Test for dynamically changing the number of vertexes (points)
	// This is for testing the drawing for point-clouds,
	// which are of uncertain/dynamic number of points
	int _start_count = int(currentTime*1000);
	if (_start_count > NUM_STARS) _start_count=NUM_STARS;
	glDrawArrays(GL_POINTS, 0, _start_count);
	*/
	///////////////////////////
    // std::cout << "Drawing points\n";
	glutSwapBuffers();
}

//Call to resize the window
void My_Reshape(int width, int height)
{
	glViewport(0, 0, width, height);

	float viewportAspect = (float)width / (float)height;

	proj_matrix = glm::perspective(deg2rad(50.0f), viewportAspect, 0.1f, 1000.0f);
}

//Timer event
void My_Timer(int val)
{
	glutPostRedisplay();
	glutTimerFunc(16, My_Timer, val);
}

//Special key event
void My_SpecialKeys(int key, int x, int y)
{
	switch (key)
	{
	case GLUT_KEY_F1:
		printf("F1 is pressed at (%d, %d)\n", x, y);
		break;
	case GLUT_KEY_PAGE_UP:
		printf("Page up is pressed at (%d, %d)\n", x, y);
		break;
	case GLUT_KEY_LEFT:
		printf("Left arrow is pressed at (%d, %d)\n", x, y);
		break;
	default:
		printf("Other special key is pressed at (%d, %d)\n", x, y);
		break;
	}
}

int main(int argc, char *argv[])
{

    // ROS
    //------------------------------------------------//
    ROS_INTERFACE ros_interface(argc, argv);
    ros_interface_ptr = &ros_interface;

    // Total topic count
    size_t topic_count = STRING_TOPIC_COUNT;
    // Topic names
    vector<string> string_topic_names;
    for (size_t i=0; i < topic_count; ++i){
        std::stringstream _ss_topic_name;
        _ss_topic_name << "chatter_" << i;
        string_topic_names.push_back(_ss_topic_name.str());
    }
    // string_topic_names.push_back("chatter_0");




    // std::cout << "here\n";
    {
        using MSG::M_TYPE;
        // String
        for (size_t i=0; i < string_topic_names.size(); ++i){
            ros_interface.add_a_topic(string_topic_names[i], int(M_TYPE::String), true, 1000, 10);
        }
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

    // std::cout << "here\n";

    // start
    ros_interface.start();
    // std::this_thread::sleep_for( std::chrono::milliseconds(3000) );
    // std::cout << "here\n";



    // Image
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






	// Initialize GLUT and GLEW, then create a window.
	////////////////////
	glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
/*
#ifdef _MSC_VER
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
#else
	glutInitDisplayMode(GLUT_3_2_CORE_PROFILE | GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
#endif
*/
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(600, 600);
	glutCreateWindow("Framework"); // You cannot use OpenGL functions before this line; The OpenGL context must be created first by glutCreateWindow()!

	glewInit();
#ifdef _MSC_VER

#endif

	//Print debug information
	DumpInfo();
	////////////////////

	//Call custom initialize function
	My_Init();

	//Register GLUT callback functions
	////////////////////
    glutIdleFunc(My_Idle);
	glutDisplayFunc(My_Display);
	glutReshapeFunc(My_Reshape);
	glutTimerFunc(16, My_Timer, 0);
	////////////////////

	// Enter main event loop.
	// glutMainLoop();
    enter_main_loop();


	return 0;
}
