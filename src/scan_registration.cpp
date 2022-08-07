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
#include <vector>
#include <string>
#include "mloam/scan_registration.h"
#include <pcl/filters/voxel_grid.h>

// Velodyne HDL64 specific parameters 
static const double HDL_64_SCAN_PERIOD = 0.1;
static const int HDL_64_NUM_SCAN_RINGS = 64;

// How many point clouds to skip on startup, the delay
const int SYSTEM_DELAY = 0;
// How many point clouds we have skipped
static int system_delay_count = 0;

static bool is_system_initialized = false;

static constexpr int MAX_NUMBER_OF_POINTS = 400000;

static float cloud_curvatures[MAX_NUMBER_OF_POINTS];
static int cloud_sorted_indices[MAX_NUMBER_OF_POINTS];
// TODO: I think this is just a 0 or 1 to indicate point has been picked, could replace with bool?
int cloud_neighbor_picked[MAX_NUMBER_OF_POINTS];
int cloud_label[MAX_NUMBER_OF_POINTS];

// Before doing any point feature extraction we will remove any points less than this distance
static const double MINIMUM_DISTANCE_THRESHOLD = 5.0;

bool CompareCloudCurvatures(int i, int j) { 
  return (cloud_curvatures[i] < cloud_curvatures[j]); 
}

static pcl::PointCloud<pcl::PointXYZI> corner_points_sharp;
static pcl::PointCloud<pcl::PointXYZI> corner_points_less_sharp;
static pcl::PointCloud<pcl::PointXYZI> surface_points_flat;
static pcl::PointCloud<pcl::PointXYZI> surface_points_less_flat;
static pcl::PointCloud<pcl::PointXYZI> scan_point_cloud;
static std::vector<pcl::PointCloud<pcl::PointXYZI>> scan_rings(HDL_64_NUM_SCAN_RINGS);
/**
 * @brief Remove points below an input distance threshold, modifies input point cloud
 * 
 * @tparam PointT 
 * @param pointCloud 
 * @param distanceThreshold 
 */
void RemovePointsLessThanDistance(pcl::PointCloud<pcl::PointXYZI> &pointCloud,
                                  double distanceThreshold) {
  
  // We will use the distance squared as our check, avoids having to call sqrt()
  const double threshold_squared = distanceThreshold * distanceThreshold;
  size_t j = 0;
  for (size_t i = 0; i < pointCloud.points.size(); ++i) {
    double point_distance = pointCloud.points[i].x * pointCloud.points[i].x + 
                            pointCloud.points[i].y * pointCloud.points[i].y + 
                            pointCloud.points[i].z * pointCloud.points[i].z;
    if (point_distance < threshold_squared) {
      continue;
    }
    pointCloud.points[j] = pointCloud.points[i];
    j++;
  }

  // TODO: Not sure why resizing is required if we removed points, maybe free's memory?
  if (j != pointCloud.points.size()) {
    pointCloud.points.resize(j);
  }

  pointCloud.height = 1;
  pointCloud.width = static_cast<uint32_t>(j);
  pointCloud.is_dense = true;
}

void ScanRegistration_Run(const pcl::PointCloud<pcl::PointXYZI> &pointCloud) {
  scan_point_cloud = pointCloud;
  std::cout << "Input point cloud of size " << scan_point_cloud.size() << std::endl;

  if (!is_system_initialized) {
    system_delay_count += 1;
    if (system_delay_count >= SYSTEM_DELAY) {
      is_system_initialized = true;
    }
    else {
      return;
    }
  }

  scan_rings.clear();
  corner_points_less_sharp.clear();
  corner_points_sharp.clear();
  surface_points_flat.clear();
  surface_points_less_flat.clear();

  /* The main steps that are going to happen in scan registration is:
    1) Pre-filter points by distance
    2) Separate the point cloud into their respective scan rings
    3) Find point features in each scan ring
  */

  std::vector<int> scan_start_indices(HDL_64_NUM_SCAN_RINGS, 0);
  std::vector<int> scan_end_indices(HDL_64_NUM_SCAN_RINGS, 0);

  RemovePointsLessThanDistance(scan_point_cloud, MINIMUM_DISTANCE_THRESHOLD);

  int cloud_size = scan_point_cloud.points.size();
  float start_orientation = -atan2(scan_point_cloud.points[0].y, scan_point_cloud.points[0].x);
  float end_orientation = -atan2(scan_point_cloud.points[cloud_size - 1].y,
                                 scan_point_cloud.points[cloud_size - 1].x) +
                                 2 * M_PI;

  if (end_orientation - start_orientation > 3 * M_PI) {
    end_orientation -= 2 * M_PI;
  }
  else if (end_orientation - start_orientation < M_PI) {
    end_orientation += 2 * M_PI;
  }

  bool passed_halfway = false;
  int count = cloud_size;
  pcl::PointXYZI point;

  for (int i = 0; i < cloud_size; i++) {
    point.x = pointCloud.points[i].x;
    point.y = pointCloud.points[i].y;
    point.z = pointCloud.points[i].z;

    float vertical_angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
    int scan_id = 0;

    if (vertical_angle >= -8.83) {
      scan_id = int((2 - vertical_angle) * 3.0 + 0.5);
    }
    else {
      scan_id = HDL_64_NUM_SCAN_RINGS / 2 + int((-8.83 - vertical_angle) * 2.0 + 0.5);
    }

    // use [0 50]  > 50 remove outlies
    if (vertical_angle > 2 || vertical_angle < -24.33 || scan_id > 50 || scan_id < 0) {
      count--;
      continue;
    }
    
    float orientation = -atan2(point.y, point.x);
    if (!passed_halfway) {
      if (orientation < start_orientation - M_PI / 2) {
        orientation += 2 * M_PI;
      }
      else if (orientation > start_orientation + M_PI * 3 / 2) {
        orientation -= 2 * M_PI;
      }

      if (orientation - start_orientation > M_PI) {
        passed_halfway = true;
      }
    }
    else {
      orientation += 2 * M_PI;
      if (orientation < end_orientation - M_PI * 3 / 2) {
        orientation += 2 * M_PI;
      }
      else if (orientation > end_orientation + M_PI / 2) {
        orientation -= 2 * M_PI;
      }
    }

    float relative_time = (orientation - start_orientation) / (end_orientation - start_orientation);
    point.intensity = scan_id + HDL_64_SCAN_PERIOD * relative_time;
    scan_rings[scan_id].push_back(point);
  }

  cloud_size = count;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < HDL_64_NUM_SCAN_RINGS; i++) {
    scan_start_indices[i] = laser_cloud->size() + 5;
    *laser_cloud += scan_rings[i];
    scan_end_indices[i] = laser_cloud->size() - 6;
  }

  for (int i = 5; i < cloud_size - 5; i++) {
    float diffX = laser_cloud->points[i - 5].x + laser_cloud->points[i - 4].x + laser_cloud->points[i - 3].x + laser_cloud->points[i - 2].x + laser_cloud->points[i - 1].x - 10 * laser_cloud->points[i].x + laser_cloud->points[i + 1].x + laser_cloud->points[i + 2].x + laser_cloud->points[i + 3].x + laser_cloud->points[i + 4].x + laser_cloud->points[i + 5].x;
    float diffY = laser_cloud->points[i - 5].y + laser_cloud->points[i - 4].y + laser_cloud->points[i - 3].y + laser_cloud->points[i - 2].y + laser_cloud->points[i - 1].y - 10 * laser_cloud->points[i].y + laser_cloud->points[i + 1].y + laser_cloud->points[i + 2].y + laser_cloud->points[i + 3].y + laser_cloud->points[i + 4].y + laser_cloud->points[i + 5].y;
    float diffZ = laser_cloud->points[i - 5].z + laser_cloud->points[i - 4].z + laser_cloud->points[i - 3].z + laser_cloud->points[i - 2].z + laser_cloud->points[i - 1].z - 10 * laser_cloud->points[i].z + laser_cloud->points[i + 1].z + laser_cloud->points[i + 2].z + laser_cloud->points[i + 3].z + laser_cloud->points[i + 4].z + laser_cloud->points[i + 5].z;

    cloud_curvatures[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    cloud_sorted_indices[i] = i;
    cloud_neighbor_picked[i] = 0;
    cloud_label[i] = 0;
  }


  float t_q_sort = 0;
  for (int i = 0; i < HDL_64_NUM_SCAN_RINGS; i++) {
    if (scan_end_indices[i] - scan_start_indices[i] < 6) {
      continue;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr surface_points_less_flat_scan(new pcl::PointCloud<pcl::PointXYZI>);
    for (int j = 0; j < 6; j++) {
      int sp = scan_start_indices[i] + (scan_end_indices[i] - scan_start_indices[i]) * j / 6;
      int ep = scan_start_indices[i] + (scan_end_indices[i] - scan_start_indices[i]) * (j + 1) / 6 - 1;

      std::sort(cloud_sorted_indices + sp, cloud_sorted_indices + ep + 1, CompareCloudCurvatures);

      int largest_picked_number = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloud_sorted_indices[k];

        if (cloud_neighbor_picked[ind] == 0 && cloud_curvatures[ind] > 0.1) {

          largest_picked_number++;
          if (largest_picked_number <= 2) {
            cloud_label[ind] = 2;
            corner_points_sharp.push_back(laser_cloud->points[ind]);
            corner_points_less_sharp.push_back(laser_cloud->points[ind]);
          }
          else if (largest_picked_number <= 20) {
            cloud_label[ind] = 1;
            corner_points_less_sharp.push_back(laser_cloud->points[ind]);
          }
          else {
            break;
          }

          cloud_neighbor_picked[ind] = 1;

          for (int l = 1; l <= 5; l++) {
            float diffX = laser_cloud->points[ind + l].x - laser_cloud->points[ind + l - 1].x;
            float diffY = laser_cloud->points[ind + l].y - laser_cloud->points[ind + l - 1].y;
            float diffZ = laser_cloud->points[ind + l].z - laser_cloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloud_neighbor_picked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laser_cloud->points[ind + l].x - laser_cloud->points[ind + l + 1].x;
            float diffY = laser_cloud->points[ind + l].y - laser_cloud->points[ind + l + 1].y;
            float diffZ = laser_cloud->points[ind + l].z - laser_cloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloud_neighbor_picked[ind + l] = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloud_sorted_indices[k];

        if (cloud_neighbor_picked[ind] == 0 && cloud_curvatures[ind] < 0.1) {

          cloud_label[ind] = -1;
          surface_points_flat.push_back(laser_cloud->points[ind]);

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloud_neighbor_picked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = laser_cloud->points[ind + l].x - laser_cloud->points[ind + l - 1].x;
            float diffY = laser_cloud->points[ind + l].y - laser_cloud->points[ind + l - 1].y;
            float diffZ = laser_cloud->points[ind + l].z - laser_cloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloud_neighbor_picked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laser_cloud->points[ind + l].x - laser_cloud->points[ind + l + 1].x;
            float diffY = laser_cloud->points[ind + l].y - laser_cloud->points[ind + l + 1].y;
            float diffZ = laser_cloud->points[ind + l].z - laser_cloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloud_neighbor_picked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloud_label[k] <= 0) {
          surface_points_less_flat_scan->push_back(laser_cloud->points[k]);
        }
      }
    }

    pcl::PointCloud<pcl::PointXYZI> downsampled_surface_points;
    pcl::VoxelGrid<pcl::PointXYZI> down_size_voxel_filter;
    down_size_voxel_filter.setInputCloud(surface_points_less_flat_scan);
    down_size_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    down_size_voxel_filter.filter(downsampled_surface_points);

    surface_points_less_flat += downsampled_surface_points;
  }
}

// void ScanRegistration_Draw(const Camera3d *cam) {
//   if (!ImGui::Begin("Scan Registration")) {
//     ImGui::End();
//     return;
//   }

//   if (ImGui::TreeNode("Corner Points Sharp")) {
//     static bool show = true;
//     ImGui::Checkbox("Show", &show);
//     static ImVec4 color = ImVec4(0.0, 1.0, 0.0, 1.0);
//     ImGui::ColorPicker4("Point Color", &color.x);
//     static int point_size = 5;
//     ImGui::InputInt("Point Size", &point_size, 1, 1);

//     if (show) {
//       LidarViewer_Draw(corner_points_sharp, cam, color, point_size);
//     }
//     ImGui::TreePop();
//   }

//   if (ImGui::TreeNode("Corner Points Less Sharp")) {
//     static bool show = true;
//     ImGui::Checkbox("Show", &show);
//     static ImVec4 color = ImVec4(0.0, 0.0, 1.0, 1.0);
//     ImGui::ColorPicker4("Point Color", &color.x);
//     static int point_size = 5;
//     ImGui::InputInt("Point Size", &point_size, 1, 1);

//     if (show) {
//       LidarViewer_Draw(corner_points_less_sharp, cam, color, point_size);
//     }
//     ImGui::TreePop();
//   }

//   ImGui::End();
// }