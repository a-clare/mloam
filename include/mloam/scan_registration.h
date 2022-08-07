#ifndef MLOAM_SCAN_REGISTRATION_H
#define MLOAM_SCAN_REGISTRATION_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include "visualizer/camera3d.h"
// #include "mloam/lidar_viewer.h"

void ScanRegistration_Run(const pcl::PointCloud<pcl::PointXYZI> &pointCloud);
// void ScanRegistration_Draw(const Camera3d *cam);

#endif