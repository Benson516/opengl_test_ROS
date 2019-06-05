#include <ROS_interface.hpp>
//
// #include "Common.h"
#include "ViewManager.h"
#include <iostream>
#include <string>
//
#include <setjmp.h> // For leaving main loop

// test
#include <FILTER_LIB.h>

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

// #define __DEBUG__
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


// Declare outside the loop to avoid periodically construction and destruction.
std::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > pc_out_ptr;
//


//uniform id

struct
{
	GLint  view_matrix;
	GLint  proj_matrix;
} uniforms;

// The structure for each star
struct star_t
{
    glm::vec3     position;
    glm::vec3     color;
};



//
ViewManager		m_camera;
float			m_zoom = 3.0f;
float			aspect;
vec2			m_screenSize;
//
GLuint			program;			//shader program

mat4			proj_matrix;		//projection matrix
GLuint          vao;
GLuint			vertex_shader;
GLuint			fragment_shader;
GLuint          buffer;
GLint           mv_location;
GLint           proj_location;
GLint			time_Loc;

// test, off-screen rendering
GLuint			program2;             // Shader program for off-screen rendering
GLuint          window_vao;
GLuint			window_buffer;
GLuint			FBO;
GLuint			depthRBO;
GLuint			FBODataTexture;
/*
static const GLfloat window_positions[] =
{
    // Position i, texcord i
	1.0f,-1.0f,1.0f,0.0f,  // right-down
	-1.0f,-1.0f,0.0f,0.0f, // left-down
	-1.0f,1.0f,0.0f,1.0f,  // left-up
	1.0f,1.0f,1.0f,1.0f    // right-up
};
*/
static const GLfloat window_positions[] =
{
    // Position i, texcord i
	1.0f,-0.0f,1.0f,0.0f,  // right-down
	-1.0f,-0.0f,0.0f,0.0f, // left-down
	-1.0f,1.0f,0.0f,1.0f,  // left-up
	1.0f,1.0f,1.0f,1.0f    // right-up
};
//


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
	char** vsSource = Common::LoadShaderSource(path_vs.c_str());
	char** fsSource = Common::LoadShaderSource(path_fs.c_str());

    std::cout << "here\n";

	glShaderSource(vs, 1, vsSource, NULL);
	glShaderSource(fs, 1, fsSource, NULL);
	Common::FreeShaderSource(vsSource);
	Common::FreeShaderSource(fsSource);
	glCompileShader(vs);
	glCompileShader(fs);
	Common::ShaderLog(vs);
	Common::ShaderLog(fs);

	//Attach Shader to program
	glAttachShader(program, vs);
	glAttachShader(program, fs);
	glLinkProgram(program);

    //Cache uniform variable id
	uniforms.view_matrix = glGetUniformLocation(program, "view");
	uniforms.proj_matrix = glGetUniformLocation(program, "projection");

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	proj_location = glGetUniformLocation(program, "proj_matrix");
	time_Loc = glGetUniformLocation(program, "time");

	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

    /*
	struct star_t
	{
		glm::vec3     position;
		glm::vec3     color;
	};
    */

	glGenBuffers(1, &buffer);
	glBindBuffer(GL_ARRAY_BUFFER, buffer);
	// glBufferData(GL_ARRAY_BUFFER, NUM_STARS * sizeof(star_t), NULL, GL_STATIC_DRAW);
	glBufferData(GL_ARRAY_BUFFER, NUM_STARS * sizeof(star_t), NULL, GL_DYNAMIC_DRAW); // test, change to dynamic draw to assign point cloud

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
	TextureData tdata = Common::Load_png(path_tdata.c_str());

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glGenTextures(1, &m_texture);
	glBindTexture(GL_TEXTURE_2D, m_texture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tdata.width, tdata.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, tdata.data);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);


    // test, off-screen rendering
    // Initialize shader2 for 2D rendering
	//----------------------------------------------------------//
	program2 = glCreateProgram();

	GLuint vs2 = glCreateShader(GL_VERTEX_SHADER);
	GLuint fs2 = glCreateShader(GL_FRAGMENT_SHADER);
    std::string path_vs2("Assets/7.5.Point_Sprite/gray.vs.glsl");
    std::string path_fs2("Assets/7.5.Point_Sprite/gray.fs.glsl");
    path_vs2 = path_pkg_directory + path_vs2;
    path_fs2 = path_pkg_directory + path_fs2;
    char** vsSource2 = Common::LoadShaderSource(path_vs2.c_str());
	char** fsSource2 = Common::LoadShaderSource(path_fs2.c_str());
	glShaderSource(vs2, 1, vsSource2, NULL);
	glShaderSource(fs2, 1, fsSource2, NULL);
	Common::FreeShaderSource(vsSource2);
	Common::FreeShaderSource(fsSource2);
	glCompileShader(vs2);
	glCompileShader(fs2);
	Common::ShaderLog(vs2);
	Common::ShaderLog(fs2);

	//Attach Shader to program
	glAttachShader(program2, vs2);
	glAttachShader(program2, fs2);
	glLinkProgram(program2);

	glGenVertexArrays(1, &window_vao);
	glBindVertexArray(window_vao);

	glGenBuffers(1, &window_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, window_buffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(window_positions), window_positions, GL_STATIC_DRAW);

	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(GL_FLOAT) * 4, 0);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(GL_FLOAT) * 4, (const GLvoid*)(sizeof(GL_FLOAT) * 2));

	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
    //----------------------------------------------------------//

    // test, off-screen rendering
    glGenFramebuffers(1, &FBO);
    //
}

// GLUT callback. Called to draw the scene.
void My_Idle(){
    // Update screen later
    // glutPostRedisplay();
}
// GLUT callback. Called to draw the scene.
size_t num_points = NUM_STARS;
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

    ROS_INTERFACE &ros_interface = (*ros_interface_ptr);
    //
    if (!ros_interface.is_running()){
        std::cout << "Leaving main loop\n";
        leave_main_loop();
        std::cout << "Quit main loop\n";
    }

    // Image
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
            imshow(window_names[i], image_out_list[i]);
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
    /*
    for (size_t i=0; i < num_pointcloud; ++i){
        bool result = ros_interface.get_ITRIPointCloud( (ITRIPointCloud_topic_id+i), pc_out);
        //
        if (result){
            // std::cout << "got one pointcloud\n";
        }
    }
    */

    // Nomatter got point cloud or not, bind the buffer and vao
    glBindBuffer(GL_ARRAY_BUFFER, buffer); // Start to use the buffer
    glBindVertexArray(vao);
    //
    if (pc_result){
        // test, time
        //--------------------------//
        auto t_pc_new = std::chrono::high_resolution_clock::now();
        auto pc_period = t_pc_new - t_pc_old;
        t_pc_old = t_pc_new;
        pc_period_us = std::chrono::duration_cast<std::chrono::microseconds>(pc_period).count();
        pc_period_lpf_0.filter(pc_period_us);
        //--------------------------//

        star_t * star = (star_t *)glMapBufferRange(GL_ARRAY_BUFFER, 0, NUM_STARS * sizeof(star_t), GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
        // num_points = pc_out.width;
        num_points = pc_out_ptr->width;
        //
    	for (size_t i = 0; i < num_points; i++)
    	{
    		// star[i].position[0] = pc_out.points[i].x;
    		// star[i].position[1] = pc_out.points[i].y;
    		// star[i].position[2] = pc_out.points[i].z;
            star[i].position[0] = pc_out_ptr->points[i].x;
    		star[i].position[1] = pc_out_ptr->points[i].y;
    		star[i].position[2] = pc_out_ptr->points[i].z;
    		star[i].color[0] = 0.8f;
    		star[i].color[1] = 0.8f;
    		star[i].color[2] = 0.8f;
    	}
    	glUnmapBuffer(GL_ARRAY_BUFFER);
    }


#endif

    glBindFramebuffer(GL_FRAMEBUFFER, FBO);
    {
        //--------------------------------------------//
    	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    	//Update shaders' input variable
    	///////////////////////////
    	static const GLfloat black[] = { 0.0f, 0.0f, 0.0f, 0.0f };
    	static const GLfloat one = 1.0f;

        // glClearColor(0, 0, 0, 0);
    	glClearBufferfv(GL_COLOR, 0, black);
    	glClearBufferfv(GL_DEPTH, 0, &one);


        //
        // m_camera.SetZoom(m_zoom); // <-- No need to do this, since the m_camera already keep the zoom
        glUseProgram(program);
        {
            /*
            mat4 Identy_Init(1.0);
    	    mat4 model_matrix = Identy_Init;
            // model_matrix[3][2] = -10.0; // Note: the glm::mat4 use [col][raw] index instead of [raw][col]
    	    model_matrix = translate(model_matrix, vec3(0.0f, 0.0f, -4.0f));
            model_matrix = rotate(model_matrix, deg2rad(90.0f), vec3(0.0f, 0.0f, 1.0f)); // z-axis
            model_matrix = rotate(model_matrix, deg2rad(60.0f), vec3(0.0f, 1.0f, 0.0f)); // y-axis
            // model_matrix = rotate(model_matrix, deg2rad(0.0f), vec3(1.0f, 0.0f, 0.0f)); // x-axis
            //
            // Note: the glm::translate and glm::rotate are doing the "post-multiplication", which is different than the normal "pre-multiplication"
            mat4 view_matrix = Identy_Init;
            // view_matrix = translate(view_matrix, vec3(0.0f, 0.0f, -10.0f));
            // view_matrix = rotate(view_matrix, deg2rad(90.0f), vec3(0.0f, 0.0f, 1.0f)); // z-axis
            // view_matrix = rotate(view_matrix, deg2rad(60.0f), vec3(0.0f, 1.0f, 0.0f)); // y-axis
            // view_matrix = rotate(view_matrix, deg2rad(0.0f), vec3(1.0f, 0.0f, 0.0f)); // x-axis
            // mat4 view_matrix_inv = inverse(view_matrix);
            //
            glUniformMatrix4fv(uniforms.view_matrix, 1, GL_FALSE, value_ptr(view_matrix*model_matrix) );
            */

            glUniformMatrix4fv(uniforms.view_matrix, 1, GL_FALSE, value_ptr(m_camera.GetViewMatrix() * m_camera.GetModelMatrix()));
        	glUniformMatrix4fv(uniforms.proj_matrix, 1, GL_FALSE, value_ptr(m_camera.GetProjectionMatrix(aspect)));
            //
            // Point sprite
            //--------------------------------//
        	glEnable(GL_POINT_SPRITE);
            // Assign the blending rule
        	glEnable(GL_BLEND);
        	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            // glBlendFunc(GL_SRC_ALPHA, GL_ONE);
            //
        	glActiveTexture(GL_TEXTURE0);
        	glBindTexture(GL_TEXTURE_2D, m_texture);
        	glEnable(GL_PROGRAM_POINT_SIZE);
         	// glDrawArrays(GL_POINTS, 0, NUM_STARS); // <-- draw all the points
            // test, to draw a partial of points
            glDrawArrays(GL_POINTS, 0, num_points); // draw part of points
            // Close
            glDisable(GL_BLEND);
            glDisable(GL_POINT_SPRITE);
            //--------------------------------//
        }
        glUseProgram(0);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);



    // test, off-screen rendering
    //-----------------------------------------------------//
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
    //

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, FBODataTexture);
    glBindBuffer(GL_ARRAY_BUFFER, window_buffer);
    glBindVertexArray(window_vao);
    glUseProgram(program2);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    // glUseProgram(0);
	//-----------------------------------------------------//


    // end of drawing
	glutSwapBuffers();



    //=============================================================//
    // end Evaluation

    //
    auto elapsed = std::chrono::high_resolution_clock::now() - start;
    auto period = start - start_old;
    start_old = start;


    long long elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
    long long period_us = std::chrono::duration_cast<std::chrono::microseconds>(period).count();

#ifdef __DEBUG__
    std::cout << "execution time (ms): " << elapsed_us*0.001 << ",\t";
    std::cout << "Image[0] rate: " << (1000000.0/image_period_lpf_0.output) << " fps\t";
    std::cout << "PointCloud rate: " << (1000000.0/pc_period_lpf_0.output) << " fps\t";
    std::cout << "\n";
    //
#endif


}

//Call to resize the window
void My_Reshape(int width, int height)
{
    glViewport(0, 0, width, height);

    //
    int width_1, height_1;
    width_1 = width;
    height_1 = height/2;
    //
    // m_screenSize = vec2(width_1, height_1);
	// aspect = width_1 * 1.0f / height_1;
    aspect = width * 1.0f / height; // <-- Why using this is correct?
	m_camera.SetWindowSize(width_1, height_1);

    // glViewport(0, 0, width_1, height_1);


    // test, off-screen rendering
    //--------------------------------------------//
    int width_2, height_2;
    width_2 = width;
    height_2 = height;
    // Clear buffer
    glDeleteRenderbuffers(1, &depthRBO);
	glDeleteTextures(1, &FBODataTexture);
    // generate RBO
    glGenRenderbuffers(1, &depthRBO);
	glBindRenderbuffer(GL_RENDERBUFFER, depthRBO);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, width_1, height_1);
    // Generate FBO texture
    glGenTextures(1, &FBODataTexture);
	glBindTexture(GL_TEXTURE_2D, FBODataTexture);
        // Tesxture settings
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width_1, height_1, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    // unbind
    glBindTexture(GL_TEXTURE_2D, 0);
    // Connect FBO and RBO/FBODataTexture
    glBindFramebuffer(GL_FRAMEBUFFER, FBO);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthRBO); // Connect RBO to FBO
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, FBODataTexture, 0); // Connect FBODataTexture to FBO

    //--------------------------------------------//

}

//Timer event
void My_Timer(int val)
{
	glutPostRedisplay();
	glutTimerFunc(16, My_Timer, val);
}

//
void My_Keyboard(unsigned char key, int x, int y)
{
	if (key == 27){ // Escape key
		exit(0);
	}
    m_camera.keyEvents(key);
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

void My_Mouse(int button, int state, int x, int y)
{
    m_camera.mouseEvents(button, state, x, y);
    // m_zoom = m_camera.GetZoom(); // No need to do this, since m_camera already keep the zoom
    // TwRefreshBar(bar);
}
void My_Mouse_Moving(int x, int y) {
	m_camera.mouseMoveEvent(x, y);
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
	glutInitWindowSize(800, 600);
	glutCreateWindow("Framework"); // You cannot use OpenGL functions before this line; The OpenGL context must be created first by glutCreateWindow()!

	glewInit();
#ifdef _MSC_VER

#endif

	//Print debug information
	Common::DumpInfo();
	////////////////////

	//Call custom initialize function
	My_Init();

	//Register GLUT callback functions
	////////////////////
    // glutIdleFunc(My_Idle); // <-- Note: If overwrite this function, remember to add a sleep() call.
	glutDisplayFunc(My_Display);
	glutReshapeFunc(My_Reshape);
	glutTimerFunc(16, My_Timer, 0);
    //
    glutKeyboardFunc(My_Keyboard);
	glutSpecialFunc(My_SpecialKeys);
    //
    glutMouseFunc(My_Mouse);
    glutPassiveMotionFunc(My_Mouse_Moving);
	glutMotionFunc(My_Mouse_Moving);

	////////////////////

	// Enter main event loop.
	// glutMainLoop();
    enter_main_loop();


	return 0;
}
