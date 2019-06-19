#ifndef RM_BOUNDINGBOX_2D_H
#define RM_BOUNDINGBOX_2D_H

#include "rmBaseModel.h"



class rmBoundingBox2D : public rmBaseModel
{
public:
    rmBoundingBox2D(std::string _path_Assets_in, int _ROS_topic_id_in);
    //
	void Update(float dt);
    void Update(ROS_INTERFACE &ros_interface);
    void Update(ROS_API &ros_api);
	void Render(std::shared_ptr<ViewManager> _camera_ptr);

    void setup_params(int im_width_in, int im_height_in, int box_offset_in_image_cv_x_in, int box_offset_in_image_cv_y_in){
        im_width = im_width_in;
        im_height = im_height_in;
        im_aspect = float(im_width) / float(im_height);
        box_offset_in_image_cv_x = box_offset_in_image_cv_x_in;
        box_offset_in_image_cv_y = box_offset_in_image_cv_y_in;
    }

protected:
    void Init();
    virtual void LoadModel();
    //
    int _ROS_topic_id;
    std::shared_ptr< msgs::CamObj > msg_out_ptr;
    // ros::Time msg_time;

    void update_GL_data();

private:
    // model info
    struct Shape{
        GLuint vao;
        GLuint vbo;
        GLuint ebo;
        GLuint m_texture;
        //
        int indexCount;

        glm::mat4 model;
    };
    Shape m_shape;

    // The structure for point
    struct vertex_p_c_2D
	{
		glm::vec2     position;
		glm::vec3     color;
	};
    int _num_vertex_idx_per_box;
    long long _max_num_vertex_idx;
    int _num_vertex_per_box;
    long long _max_num_vertex;
    long long _max_num_box;


    // OpenCV --> OpenGL
    //-------------------------------------------------------//
    // The box param in openCV style
    struct box_param_cv{
        glm::vec2 xy;
        int width;
        int height;
        int obj_class;

        box_param_cv(int x, int y, int w, int h, int obj_class_in):
            xy(x, y), width(w), height(h), obj_class(obj_class_in)
        {}
    };

    // The box param in openGL style: 4 points in normalized coordinate: x,y belongs to [-1, 1]
    struct box_param_gl{
        glm::vec2 xy_list[4];
        int obj_class;
    };

    // The image params
    // Note: The origin of the image is at its center.
    int im_width;
    int im_height;
    float im_aspect; // w / h
    // The box coordinate relative to image, normally (0,0)
    int box_offset_in_image_cv_x;
    int box_offset_in_image_cv_y;



    inline void toNormGL(int cv_x, int cv_y, float &gl_x, float &gl_y){
        // Convert CV coordinate to normalized GL coordinate
        gl_x = (cv_x + box_offset_in_image_cv_x)/float(im_width) * 2.0 - 1.0;
        gl_y = (cv_y + box_offset_in_image_cv_y)/float(im_height) * -2.0 + 1.0;
    }
    void convert_cv_to_normalized_gl(const box_param_cv &box_cv_in, box_param_gl & box_gl_out){
        box_gl_out.obj_class = box_cv_in.obj_class;
        float gl_x, gl_y, gl_w, gl_h;
        gl_w = box_cv_in.width/float(im_width);
        gl_h = box_cv_in.height/float(im_height);
        int _i = 0;
        // P1
        toNormGL(box_cv_in.xy[0], box_cv_in.xy[1], gl_x, gl_y);
        box_gl_out.xy_list[_i++] = glm::vec2(gl_x, gl_y);
        // P2
        toNormGL(box_cv_in.xy[0]+gl_w, box_cv_in.xy[1], gl_x, gl_y);
        box_gl_out.xy_list[_i++] = glm::vec2(gl_x, gl_y);
        // P3
        toNormGL(box_cv_in.xy[0]+gl_w, box_cv_in.xy[1]+gl_h, gl_x, gl_y);
        box_gl_out.xy_list[_i++] = glm::vec2(gl_x, gl_y);
        // P4
        toNormGL(box_cv_in.xy[0], box_cv_in.xy[1]+gl_h, gl_x, gl_y);
        box_gl_out.xy_list[_i++] = glm::vec2(gl_x, gl_y);
    }
    //-------------------------------------------------------//



    //uniform id
	struct
	{
		GLint  mv_matrix;
		GLint  proj_matrix;
	} uniforms;

    std::vector<vertex_p_c_2D> box_template;

    inline void generate_box_template(){
        box_template.resize(4);
        box_template[0].position = glm::vec2(0.0f, 0.0f);
        box_template[1].position = glm::vec2(1.0f, 0.0f);
        box_template[2].position = glm::vec2(1.0f, 1.0f);
        box_template[3].position = glm::vec2(0.0f, 1.0f);

    }

};

#endif // RM_BOUNDINGBOX_2D_H
