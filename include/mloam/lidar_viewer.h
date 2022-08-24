#ifndef LIDAR_VIEWER_H
#define LIDAR_VIEWER_H

#include "mgl/mgl.h"
#include "mgl/camera3d.h"
#include "imgui.h"
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mloam {

void LidarViewer_Setup();

void LidarViewer_Draw(const pcl::PointCloud<pcl::PointXYZI> &scan,
                      const mgl::Camera3D &cam,
                      const ImVec4 color,
                      int pointSize);

} // end of namespace mloam
#endif