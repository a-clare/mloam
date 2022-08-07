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
#include "mloam/mloam.h"
#include <pcl/filters/voxel_grid.h>

// Velodyne HDL64 specific parameters 
static const double HDL64_SCAN_PERIOD = 0.1;
static const int HDL64_NUM_SCAN_RINGS = 64;
static const int HDL64_NUM_LASERS_PER_BLOCK = 32;
static const float HDL64_UPPER_BLOCK_MIN_ANGLE = -8.33f;
static const float HDL64_UPPER_BLOCK_MAX_ANGLE = 2.0f;
static const float HDL64_MIN_ANGLE = -24.33;

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

// As part of the scan registration we will separate each point into its individual
// scan rings (0 to 64 for HDl64). This is an intermediate step in the process so
// we will re-use scan_rings every iteration 
static std::vector<pcl::PointCloud<pcl::PointXYZI>> scan_rings(HDL64_NUM_SCAN_RINGS);

bool CompareCloudCurvatures(int i, int j) { 
  return (cloud_curvatures[i] < cloud_curvatures[j]); 
}

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

bool mloam::ScanRegistration(const pcl::PointCloud<pcl::PointXYZI> &inputCloud,
                             pcl::PointCloud<pcl::PointXYZI> &cornerPointsSharp,
                             pcl::PointCloud<pcl::PointXYZI> &cornerPointsLessSharp,
                             pcl::PointCloud<pcl::PointXYZI> &surfacePointsFlat,
                             pcl::PointCloud<pcl::PointXYZI> &surfacePointsLessFlat,
                             pcl::PointCloud<pcl::PointXYZI> &filteredCloud) {
  
  /* The main steps of the scan registration are:
    1) Pre-filter points by distance
    2) Separate the point cloud into their respective scan rings, 
    3) Find point features in each scan ring by looking at the curvature of each point
  */

  filteredCloud = inputCloud;
  LOG_INFO << "Number of points in input point cloud size: " << inputCloud.size(); 

  if (!is_system_initialized) {
    system_delay_count += 1;
    if (system_delay_count >= SYSTEM_DELAY) {
      is_system_initialized = true;
    }
    else {
      return false;
    }
  }

  // Reset each ring since we re-use it every iteration
  for (auto &ring : scan_rings) {
    ring.clear();
  }
  // Reset our inputs in case caller re-uses the variables, dont want to add 
  // new features to the previous iteration
  cornerPointsLessSharp.clear();
  cornerPointsSharp.clear();
  surfacePointsFlat.clear();
  surfacePointsLessFlat.clear();

  LOG_DEBUG << "Filtering points by distance";
  RemovePointsLessThanDistance(filteredCloud, MINIMUM_DISTANCE_THRESHOLD);
  LOG_INFO << "Number of points after distance filtering: " << filteredCloud.size();

  int cloud_size = filteredCloud.points.size();
  /* We assume the lidar is spinning. If we look at a single scan line, the 2nd point in the scan occurs
   * N seconds after the first point. We can estimate determine N by calculating the horizontal angle
   * (orientation) of the point and using the lidar spin rate.
   * So we need the start and end orientation of the point cloud to make all the times relative to
   * this start and end
   * TODO: This is only required if we need to undistort the point cloud, if its already
   *       corrected we dont need to do this.
   * TODO: Does this assume the lidar is reasonably level? What happens if the lidar is not level? */
  float start_orientation = -atan2(filteredCloud.points[0].y, filteredCloud.points[0].x);
  float end_orientation = -atan2(filteredCloud.points[cloud_size - 1].y,
                                 filteredCloud.points[cloud_size - 1].x) +
                                 2 * M_PI;

  // TODO: Understand what this is doing. I think its capping upper and lower angle values but need to confirm
  if (end_orientation - start_orientation > 3 * M_PI) {
    end_orientation -= 2 * M_PI;
  }
  else if (end_orientation - start_orientation < M_PI) {
    end_orientation += 2 * M_PI;
  }

  bool passed_halfway = false;
  for (auto &point : filteredCloud) {
    /* Note: At this point some loam implementations will take the point and 
             rotate it, to change its coordinate system.
             I think its only a matter of preference so im not rotating here 
    */

    float vertical_angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
    int scan_id = 0;

    // Removing outliers by checking if the vertical angle is not within the expected range
    // of the HDL64
    if (vertical_angle > HDL64_UPPER_BLOCK_MAX_ANGLE || vertical_angle < HDL64_MIN_ANGLE) {
      cloud_size--;
      continue;
    }
    
    /* The HDL64 separates the 64 lasers into an upper block and lower block, each block has 32 lasers.
     * It has a total vertical resolution of +2 degrees to -24.8 degrees.
     * However, each block has a different vertical angle separation.
     * The upper block, from [-8.33 to +2] has a 1/3 degree separation
     * The lower block, from [-24.33 to -8.33 has a 1/2 degree separation
     * AKA its not uniform (each ring is not equally spaced), so in order to separate the point cloud 
     * into scan rings we need to check if we are in the upper or lower block first
     */
    if (vertical_angle >= HDL64_UPPER_BLOCK_MIN_ANGLE) {
      scan_id = int((HDL64_UPPER_BLOCK_MAX_ANGLE - vertical_angle) * 3.0 + 0.5);
    }
    else {
      scan_id = HDL64_NUM_LASERS_PER_BLOCK + int((HDL64_UPPER_BLOCK_MIN_ANGLE - vertical_angle) * 2.0 + 0.5);
    }

    // Capping the scan rings to only include rings [0 to 50] to help remove outliers
    if (scan_id > 50 || scan_id < 0) {
      cloud_size--;
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

    // TODO: This is quite hacky, using the intensity field to hold the point scan time information
    float relative_time = (orientation - start_orientation) / (end_orientation - start_orientation);
    point.intensity = scan_id + HDL64_SCAN_PERIOD * relative_time;
    scan_rings[scan_id].push_back(point);
  }

  LOG_INFO << "Number of points after filtering points by vertical angle: " << cloud_size;

  std::vector<int> scan_start_indices(HDL64_NUM_SCAN_RINGS, 0);
  std::vector<int> scan_end_indices(HDL64_NUM_SCAN_RINGS, 0);

  // TODO: Why use a pcl pointer type here?
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < HDL64_NUM_SCAN_RINGS; i++) {
    scan_start_indices[i] = laser_cloud->size() + 5;
    *laser_cloud += scan_rings[i];
    // TOOD: I think this will cause issues if the first N rings contain 0 points. Will
    //       be doing size() - 6, which is 0 - 7, and size() is an unsigned type and might
    //       cause overflow. Investigate
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

  for (int i = 0; i < HDL64_NUM_SCAN_RINGS; i++) {
    if (scan_end_indices[i] - scan_start_indices[i] < 6) {
      continue;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfacePointsLessFlat_scan(new pcl::PointCloud<pcl::PointXYZI>);
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
            cornerPointsSharp.push_back(laser_cloud->points[ind]);
            cornerPointsLessSharp.push_back(laser_cloud->points[ind]);
          }
          else if (largest_picked_number <= 20) {
            cloud_label[ind] = 1;
            cornerPointsLessSharp.push_back(laser_cloud->points[ind]);
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
          surfacePointsFlat.push_back(laser_cloud->points[ind]);

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
          surfacePointsLessFlat_scan->push_back(laser_cloud->points[k]);
        }
      }
    }

    pcl::PointCloud<pcl::PointXYZI> downsampled_surface_points;
    pcl::VoxelGrid<pcl::PointXYZI> down_size_voxel_filter;
    down_size_voxel_filter.setInputCloud(surfacePointsLessFlat_scan);
    down_size_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    down_size_voxel_filter.filter(downsampled_surface_points);

    surfacePointsLessFlat += downsampled_surface_points;
  }
  return true;
}