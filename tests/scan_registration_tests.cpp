#ifndef MLOAM_SCAN_REGISTRATION_TESTS_H_
#define MLOAM_SCAN_REGISTRATION_TESTS_H_

#include "gtest/gtest.h"
#include "mloam/mloam.h"
#include <pcl/filters/voxel_grid.h>

TEST(ScanRegistration, CorrectNumberOfFeatures) {
  pcl::PointCloud<pcl::PointXYZI> point_cloud;

  // For scan registration
  pcl::PointCloud<pcl::PointXYZI> corner_points_sharp;
  pcl::PointCloud<pcl::PointXYZI> corner_points_less_sharp;
  pcl::PointCloud<pcl::PointXYZI> surface_points_flat;
  pcl::PointCloud<pcl::PointXYZI> surface_points_less_flat;
  pcl::PointCloud<pcl::PointXYZI> filtered_point_cloud;
  
  int binary_file_num = 1;
  point_cloud = mloam::LoadKittiData("/Users/adamclare/data/2011_09_30/2011_09_30_drive_0028_sync/velodyne_points/data/", binary_file_num);
  mloam::ScanRegistration(point_cloud, 
                          corner_points_sharp, 
                          corner_points_less_sharp, 
                          surface_points_flat,
                          surface_points_less_flat,
                          filtered_point_cloud);
  
  // These true values are taken from running original ROS based ALOAM
  EXPECT_EQ(filtered_point_cloud.size(), 82933);
  EXPECT_EQ(corner_points_sharp.size(), 597);
  EXPECT_EQ(corner_points_less_sharp.size(), 5118);
  EXPECT_EQ(surface_points_flat.size(), 1220);
  EXPECT_EQ(surface_points_less_flat.size(), 25135);

  binary_file_num += 1;
  point_cloud = mloam::LoadKittiData("/Users/adamclare/data/2011_09_30/2011_09_30_drive_0028_sync/velodyne_points/data/", binary_file_num);
  mloam::ScanRegistration(point_cloud, 
                          corner_points_sharp, 
                          corner_points_less_sharp, 
                          surface_points_flat,
                          surface_points_less_flat,
                          filtered_point_cloud);
  EXPECT_EQ(filtered_point_cloud.size(), 84323);
  EXPECT_EQ(corner_points_sharp.size(), 600);
  EXPECT_EQ(corner_points_less_sharp.size(), 5162);
  EXPECT_EQ(surface_points_flat.size(), 1222);
  EXPECT_EQ(surface_points_less_flat.size(), 25483);
}

TEST(ScanRegistration, VoxelFilter) {
  pcl::PointCloud<pcl::PointXYZI> point_cloud;

  int binary_file_num = 0;
  point_cloud = mloam::LoadKittiData("/Users/adamclare/data/2011_09_30/2011_09_30_drive_0028_sync/velodyne_points/data/", binary_file_num);

  pcl::PointCloud<pcl::PointXYZI> downsampled_surface_points;
  pcl::VoxelGrid<pcl::PointXYZI> down_size_voxel_filter;
  down_size_voxel_filter.setInputCloud(point_cloud.makeShared());
  down_size_voxel_filter.setDownsampleAllData(false);
  down_size_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
  down_size_voxel_filter.filter(downsampled_surface_points);

//  EXPECT_EQ(downsampled_surface_points.size(), 0);
//  EXPECT_EQ(point_cloud.size(), 0);
//  EXPECT_EQ(downsampled_surface_points[0].x, 0);
//  EXPECT_EQ(downsampled_surface_points[0].y, 0);
//  EXPECT_EQ(downsampled_surface_points[0].z, 0);
  std::cout << downsampled_surface_points[0] << std::endl;
  std::cout << downsampled_surface_points[20851] << std::endl;

}

#endif