#ifndef RM_POINTCLOUD_H
#define RM_POINTCLOUD_H

#include "rmBaseModel.h"

class rmPointCloud : public rmBaseModel
{
public:
    rmPointCloud(std::string _path_Assets_in);
    //
	void Update(float dt);
	void Render(std::shared_ptr<ViewManager> _camera_ptr);

protected:
    void Init();
    virtual void LoadModel();

private:
    // model info
    struct Shape{
        GLuint vao;
        GLuint vbo;
        int indexCount;
        GLuint m_texture;

        glm::mat4 model;
    };
    Shape m_shape;

    // The structure for point
    struct star_t
	{
		glm::vec3     position;
		glm::vec3     color;
	};
    long long _num_points;

    //uniform id
	struct
	{
		GLint  mv_matrix;
		GLint  proj_matrix;
	} uniforms;

    static inline float random_float()
    {
        static unsigned int seed = 0x13371337;
    	float res;
    	unsigned int tmp;
    	seed *= 16807;
    	tmp = seed ^ (seed >> 4) ^ (seed << 15);
    	*((unsigned int *)&res) = (tmp >> 9) | 0x3F800000;
    	return (res - 1.0f);
    }
};

#endif
