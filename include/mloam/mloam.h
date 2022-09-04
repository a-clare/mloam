/*
  MIT License

  Copyright (c) 2022 Adam Clare (Adam The Canadian)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

/**
 * @brief File contains all the function declerations for running mloam
 * Implementation/definitions are in separate cpp files to make separation of concern easier.
 * Also easier to work on a specific piece have not have to much clutter when first learning/implementating
 */
#ifndef MLOAM_MLOAM_H_
#define MLOAM_MLOAM_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "mloam/logger.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

namespace mloam {

struct Pose {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

struct PoseWithCovariance {
  Pose pose;
  Eigen::MatrixXd covariance = Eigen::MatrixXd(6, 6);
};

struct Twist {
  Eigen::Vector3d linear;
  Eigen::Vector3d angular;
};

struct TwistWithCovariance {
  Twist twist;
  Eigen::MatrixXd covariance = Eigen::MatrixXd(6, 6);
};

struct OdometryData {
  PoseWithCovariance pose;
  TwistWithCovariance twist;
};


/**
 * @brief Extract features from the input point cloud
 * The names of the input variables were used to try and keep it close to original
 * implementation, make it easier to follow along side original implementation.
 * In the original implementation the feature point clouds are published as ros topics
 * 
 * Extracting features will do some filtering of points to try and remove outliers, the
 * filtered pointcloud that is used is returned as filteredCloud 
 * 
 * This only works on HDL64 lidars!!!
 * 
 * @param pointCloud unmodified original point cloud that will be used to find features
 * @param cornerPointsSharp 
 * @param cornerPointsLessSharp 
 * @param surfacePointsFlat 
 * @param surfacePointsLessFlat 
 * @param filteredCloud
 * @return true if successful
 */
bool ScanRegistration(const pcl::PointCloud<pcl::PointXYZI> &inputCloud,
                      pcl::PointCloud<pcl::PointXYZI> &cornerPointsSharp,
                      pcl::PointCloud<pcl::PointXYZI> &cornerPointsLessSharp,
                      pcl::PointCloud<pcl::PointXYZI> &surfacePointsFlat,
                      pcl::PointCloud<pcl::PointXYZI> &surfacePointsLessFlat,
                      pcl::PointCloud<pcl::PointXYZI> &filteredCloud);

bool Odometry(const pcl::PointCloud<pcl::PointXYZI> &cornerPointsSharp2,
              const pcl::PointCloud<pcl::PointXYZI> &cornerPointsLessSharp2,
              const pcl::PointCloud<pcl::PointXYZI> &surfPointsFlat2,
              const pcl::PointCloud<pcl::PointXYZI> &surfPointsLessFlat2,
              const pcl::PointCloud<pcl::PointXYZI> &laserCloudFullRes2,
              OdometryData& odometryData);

void Mapping(const pcl::PointCloud<pcl::PointXYZI>& lastCloudCornerlast,
             const pcl::PointCloud<pcl::PointXYZI>& laserCloudSurfaceLast,
             const mloam::OdometryData& laserOdomToinit,
             pcl::PointCloud<pcl::PointXYZI>& laserCloudFullRes);
} // end of namespace mloam

#endif