#ifndef MLOAM_ODOMETRY_TESTS_H_
#define MLOAM_ODOMETRY_TESTS_H_

#include "gtest/gtest.h"
#include "mloam/mloam.h"


TEST(OdometryTest, CorrectPose) {

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
  
  mloam::OdometryData odom_data;
  mloam::Odometry(corner_points_sharp,
                  corner_points_less_sharp,
                  surface_points_flat,
                  surface_points_less_flat,
                  filtered_point_cloud,
                  odom_data);
  
  EXPECT_DOUBLE_EQ(odom_data.pose.pose.position.x(), 0.0);
  EXPECT_DOUBLE_EQ(odom_data.pose.pose.position.y(), 0.0);
  EXPECT_DOUBLE_EQ(odom_data.pose.pose.position.z(), 0.0);
  EXPECT_DOUBLE_EQ(odom_data.pose.pose.orientation.x(), 0.0);
  EXPECT_DOUBLE_EQ(odom_data.pose.pose.orientation.y(), 0.0);
  EXPECT_DOUBLE_EQ(odom_data.pose.pose.orientation.z(), 0.0);
  EXPECT_DOUBLE_EQ(odom_data.pose.pose.orientation.w(), 1.0);
  
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

  mloam::Odometry(corner_points_sharp,
                  corner_points_less_sharp,
                  surface_points_flat,
                  surface_points_less_flat,
                  filtered_point_cloud,
                  odom_data);
  
  EXPECT_NEAR(odom_data.pose.pose.position.x(), 0.354233, 1.0e-6);
  EXPECT_NEAR(odom_data.pose.pose.position.y(), 0.0122033, 1.0e-6);
  EXPECT_NEAR(odom_data.pose.pose.position.z(), 0.004543, 1.0e-6);
}

#endif