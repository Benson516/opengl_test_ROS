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

private:

};

#endif
