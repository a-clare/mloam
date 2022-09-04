// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <cmath>
#include "mloam/mloam.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <iostream>
#include <queue>

#include "mloam/lidar_factor.h"

#define DISTORTION 0

int corner_correspondence = 0, plane_correspondence = 0;

constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

int skipFrameNum = 5;
bool systemInited = false;

double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;

static pcl::KdTreeFLANN<pcl::PointXYZI> kdtreeCornerLast;
pcl::KdTreeFLANN<pcl::PointXYZI> kdtreeSurfLast;

static pcl::PointCloud<pcl::PointXYZI> cornerPointsSharp;
static pcl::PointCloud<pcl::PointXYZI> cornerPointsLessSharp;
static pcl::PointCloud<pcl::PointXYZI> surfPointsFlat;
static pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlat;

static pcl::PointCloud<pcl::PointXYZI> laserCloudCornerLast;
static pcl::PointCloud<pcl::PointXYZI> laserCloudSurfLast;
static pcl::PointCloud<pcl::PointXYZI> laserCloudFullRes;

static int laserCloudCornerLastNum = 0;
static int laserCloudSurfLastNum = 0;

// Transformation from current frame to world frame
static Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
static Eigen::Vector3d t_w_curr(0, 0, 0);

// q_curr_last(x, y, z, w), t_curr_last
static double para_q[4] = {0, 0, 0, 1};
static double para_t[3] = {0, 0, 0};

static Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
static Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

static int frameCount = 0;

// undistort lidar point
void TransformToStart(const pcl::PointXYZI& pi, 
                      pcl::PointXYZI& po) {
  // interpolation ratio
  double s;
  if (DISTORTION)
    s = (pi.intensity - int(pi.intensity)) / SCAN_PERIOD;
  else
    s = 1.0;
  // s = 1;
  Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
  Eigen::Vector3d t_point_last = s * t_last_curr;
  Eigen::Vector3d point(pi.x, pi.y, pi.z);
  Eigen::Vector3d un_point = q_point_last * point + t_point_last;

  po.x = un_point.x();
  po.y = un_point.y();
  po.z = un_point.z();
  po.intensity = pi.intensity;
}

// transform all lidar points to the start of the next frame

void TransformToEnd(const pcl::PointXYZI& pi, 
                    pcl::PointXYZI& po) {
  // undistort point first
  pcl::PointXYZI un_point_tmp;
  TransformToStart(pi, un_point_tmp);

  Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
  Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

  po.x = point_end.x();
  po.y = point_end.y();
  po.z = point_end.z();

  // Remove distortion time info
  po.intensity = int(pi.intensity);
}

bool mloam::Odometry(const pcl::PointCloud<pcl::PointXYZI> &cornerPointsSharp2,
                     const pcl::PointCloud<pcl::PointXYZI> &cornerPointsLessSharp2,
                     const pcl::PointCloud<pcl::PointXYZI> &surfPointsFlat2,
                     const pcl::PointCloud<pcl::PointXYZI> &surfPointsLessFlat2,
                     const pcl::PointCloud<pcl::PointXYZI> &laserCloudFullRes2,
                     OdometryData& odometryData) {
  

  cornerPointsSharp = cornerPointsSharp2;
  cornerPointsLessSharp = cornerPointsLessSharp2;
  surfPointsFlat = surfPointsFlat2;
  surfPointsLessFlat = surfPointsLessFlat2;
  laserCloudFullRes = laserCloudFullRes2;

  if (!systemInited)
  {
    systemInited = true;
    std::cout << "Initialization finished \n";
  }
  else
  {
    int cornerPointsSharpNum = cornerPointsSharp.points.size();
    int surfPointsFlatNum = surfPointsFlat.points.size();

    for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
    {
      corner_correspondence = 0;
      plane_correspondence = 0;

      // ceres::LossFunction *loss_function = NULL;
      ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
      ceres::LocalParameterization *q_parameterization =
          new ceres::EigenQuaternionParameterization();
      ceres::Problem::Options problem_options;

      ceres::Problem problem(problem_options);
      problem.AddParameterBlock(para_q, 4, q_parameterization);
      problem.AddParameterBlock(para_t, 3);

      pcl::PointXYZI pointSel;
      std::vector<int> pointSearchInd;
      std::vector<float> pointSearchSqDis;

      for (int i = 0; i < cornerPointsSharpNum; ++i)
      {
        TransformToStart(cornerPointsSharp.points[i], pointSel);
        kdtreeCornerLast.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

        int closestPointInd = -1, minPointInd2 = -1;
        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
        {
          closestPointInd = pointSearchInd[0];
          int closestPointScanID = int(laserCloudCornerLast.points[closestPointInd].intensity);

          double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
          // search in the direction of increasing scan line
          for (int j = closestPointInd + 1; j < (int)laserCloudCornerLast.points.size(); ++j)
          {
            // if in the same scan line, continue
            if (int(laserCloudCornerLast.points[j].intensity) <= closestPointScanID)
              continue;

            // if not in nearby scans, end the loop
            if (int(laserCloudCornerLast.points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
              break;

            double pointSqDis = (laserCloudCornerLast.points[j].x - pointSel.x) *
                                    (laserCloudCornerLast.points[j].x - pointSel.x) +
                                (laserCloudCornerLast.points[j].y - pointSel.y) *
                                    (laserCloudCornerLast.points[j].y - pointSel.y) +
                                (laserCloudCornerLast.points[j].z - pointSel.z) *
                                    (laserCloudCornerLast.points[j].z - pointSel.z);

            if (pointSqDis < minPointSqDis2)
            {
              // find nearer point
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          }

          // search in the direction of decreasing scan line
          for (int j = closestPointInd - 1; j >= 0; --j)
          {
            // if in the same scan line, continue
            if (int(laserCloudCornerLast.points[j].intensity) >= closestPointScanID)
              continue;

            // if not in nearby scans, end the loop
            if (int(laserCloudCornerLast.points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
              break;

            double pointSqDis = (laserCloudCornerLast.points[j].x - pointSel.x) *
                                    (laserCloudCornerLast.points[j].x - pointSel.x) +
                                (laserCloudCornerLast.points[j].y - pointSel.y) *
                                    (laserCloudCornerLast.points[j].y - pointSel.y) +
                                (laserCloudCornerLast.points[j].z - pointSel.z) *
                                    (laserCloudCornerLast.points[j].z - pointSel.z);

            if (pointSqDis < minPointSqDis2)
            {
              // find nearer point
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
          }
        }
        if (minPointInd2 >= 0) // both closestPointInd and minPointInd2 is valid
        {
          Eigen::Vector3d curr_point(cornerPointsSharp.points[i].x,
                                     cornerPointsSharp.points[i].y,
                                     cornerPointsSharp.points[i].z);
          Eigen::Vector3d last_point_a(laserCloudCornerLast.points[closestPointInd].x,
                                       laserCloudCornerLast.points[closestPointInd].y,
                                       laserCloudCornerLast.points[closestPointInd].z);
          Eigen::Vector3d last_point_b(laserCloudCornerLast.points[minPointInd2].x,
                                       laserCloudCornerLast.points[minPointInd2].y,
                                       laserCloudCornerLast.points[minPointInd2].z);

          double s;
          if (DISTORTION)
            s = (cornerPointsSharp.points[i].intensity - int(cornerPointsSharp.points[i].intensity)) / SCAN_PERIOD;
          else
            s = 1.0;
          ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
          problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
          corner_correspondence++;
        }
      }

      // find correspondence for plane features
      for (int i = 0; i < surfPointsFlatNum; ++i)
      {
        TransformToStart(surfPointsFlat.points[i], pointSel);
        kdtreeSurfLast.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
        {
          closestPointInd = pointSearchInd[0];

          // get closest point's scan ID
          int closestPointScanID = int(laserCloudSurfLast.points[closestPointInd].intensity);
          double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;

          // search in the direction of increasing scan line
          for (int j = closestPointInd + 1; j < (int)laserCloudSurfLast.points.size(); ++j)
          {
            // if not in nearby scans, end the loop
            if (int(laserCloudSurfLast.points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
              break;

            double pointSqDis = (laserCloudSurfLast.points[j].x - pointSel.x) *
                                    (laserCloudSurfLast.points[j].x - pointSel.x) +
                                (laserCloudSurfLast.points[j].y - pointSel.y) *
                                    (laserCloudSurfLast.points[j].y - pointSel.y) +
                                (laserCloudSurfLast.points[j].z - pointSel.z) *
                                    (laserCloudSurfLast.points[j].z - pointSel.z);

            // if in the same or lower scan line
            if (int(laserCloudSurfLast.points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
            {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
            // if in the higher scan line
            else if (int(laserCloudSurfLast.points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)
            {
              minPointSqDis3 = pointSqDis;
              minPointInd3 = j;
            }
          }

          // search in the direction of decreasing scan line
          for (int j = closestPointInd - 1; j >= 0; --j)
          {
            // if not in nearby scans, end the loop
            if (int(laserCloudSurfLast.points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
              break;

            double pointSqDis = (laserCloudSurfLast.points[j].x - pointSel.x) *
                                    (laserCloudSurfLast.points[j].x - pointSel.x) +
                                (laserCloudSurfLast.points[j].y - pointSel.y) *
                                    (laserCloudSurfLast.points[j].y - pointSel.y) +
                                (laserCloudSurfLast.points[j].z - pointSel.z) *
                                    (laserCloudSurfLast.points[j].z - pointSel.z);

            // if in the same or higher scan line
            if (int(laserCloudSurfLast.points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
            {
              minPointSqDis2 = pointSqDis;
              minPointInd2 = j;
            }
            else if (int(laserCloudSurfLast.points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
            {
              // find nearer point
              minPointSqDis3 = pointSqDis;
              minPointInd3 = j;
            }
          }

          if (minPointInd2 >= 0 && minPointInd3 >= 0)
          {

            Eigen::Vector3d curr_point(surfPointsFlat.points[i].x,
                                       surfPointsFlat.points[i].y,
                                       surfPointsFlat.points[i].z);
            Eigen::Vector3d last_point_a(laserCloudSurfLast.points[closestPointInd].x,
                                         laserCloudSurfLast.points[closestPointInd].y,
                                         laserCloudSurfLast.points[closestPointInd].z);
            Eigen::Vector3d last_point_b(laserCloudSurfLast.points[minPointInd2].x,
                                         laserCloudSurfLast.points[minPointInd2].y,
                                         laserCloudSurfLast.points[minPointInd2].z);
            Eigen::Vector3d last_point_c(laserCloudSurfLast.points[minPointInd3].x,
                                         laserCloudSurfLast.points[minPointInd3].y,
                                         laserCloudSurfLast.points[minPointInd3].z);

            double s;
            if (DISTORTION)
              s = (surfPointsFlat.points[i].intensity - int(surfPointsFlat.points[i].intensity)) / SCAN_PERIOD;
            else
              s = 1.0;
            ceres::CostFunction *cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
            plane_correspondence++;
          }
        }
      }

      if ((corner_correspondence + plane_correspondence) < 10)
      {
        printf("less correspondence! *************************************************\n");
      }

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.max_num_iterations = 4;
      options.minimizer_progress_to_stdout = false;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);
    }

    t_w_curr = t_w_curr + q_w_curr * t_last_curr;
    q_w_curr = q_w_curr * q_last_curr;
  }
   
  odometryData.pose.pose.orientation = q_w_curr;
  odometryData.pose.pose.position = t_w_curr;

  // transform corner features and plane features to the scan end point
  if (0)
  {
    int cornerPointsLessSharpNum = cornerPointsLessSharp.points.size();
    for (int i = 0; i < cornerPointsLessSharpNum; i++)
    {
      TransformToEnd(cornerPointsLessSharp.points[i], cornerPointsLessSharp.points[i]);
    }

    int surfPointsLessFlatNum = surfPointsLessFlat.points.size();
    for (int i = 0; i < surfPointsLessFlatNum; i++)
    {
      TransformToEnd(surfPointsLessFlat.points[i], surfPointsLessFlat.points[i]);
    }

    int laserCloudFullResNum = laserCloudFullRes.points.size();
    for (int i = 0; i < laserCloudFullResNum; i++)
    {
      TransformToEnd(laserCloudFullRes.points[i], laserCloudFullRes.points[i]);
    }
  }

  pcl::PointCloud<pcl::PointXYZI> laserCloudTemp = cornerPointsLessSharp;
  cornerPointsLessSharp = laserCloudCornerLast;
  laserCloudCornerLast = laserCloudTemp;

  laserCloudTemp = surfPointsLessFlat;
  surfPointsLessFlat = laserCloudSurfLast;
  laserCloudSurfLast = laserCloudTemp;

  laserCloudCornerLastNum = laserCloudCornerLast.points.size();
  laserCloudSurfLastNum = laserCloudSurfLast.points.size();

  // std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';

  kdtreeCornerLast.setInputCloud(laserCloudCornerLast.makeShared());
  kdtreeSurfLast.setInputCloud(laserCloudSurfLast.makeShared());

  if (frameCount % skipFrameNum == 0)
  {
    // frameCount = 0;

    // sensor_msgs::PointCloud2 laserCloudCornerLast2;
    // pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
    // laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
    // laserCloudCornerLast2.header.frame_id = "/camera";
    // pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

    // sensor_msgs::PointCloud2 laserCloudSurfLast2;
    // pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
    // laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
    // laserCloudSurfLast2.header.frame_id = "/camera";
    // pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

    // sensor_msgs::PointCloud2 laserCloudFullRes3;
    // pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
    // laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
    // laserCloudFullRes3.header.frame_id = "/camera";
    // pubLaserCloudFullRes.publish(laserCloudFullRes3);
  }
  frameCount++;
  return true;
}