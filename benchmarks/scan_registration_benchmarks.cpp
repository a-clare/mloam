#include <benchmark/benchmark.h>
#include "mloam/mloam.h"

// For scan registration
static pcl::PointCloud<pcl::PointXYZI> corner_points_sharp;
static pcl::PointCloud<pcl::PointXYZI> corner_points_less_sharp;
static pcl::PointCloud<pcl::PointXYZI> surface_points_flat;
static pcl::PointCloud<pcl::PointXYZI> surface_points_less_flat;
static pcl::PointCloud<pcl::PointXYZI> filtered_point_cloud;
static pcl::PointCloud<pcl::PointXYZI> point_cloud;

static void BM_ScanRegistration(benchmark::State& state) {
  Log::LoggingLevel() = logging::LogLevel::Error;
  int binary_file_num = (int)state.range(0);
  std::cout << "binary_file_num " << binary_file_num << std::endl;
  point_cloud = mloam::LoadKittiData("/Users/adamclare/data/2011_09_30/2011_09_30_drive_0028_sync/velodyne_points/data/", binary_file_num);

  while (state.KeepRunning()) {
    mloam::ScanRegistration(point_cloud, 
                            corner_points_sharp, 
                            corner_points_less_sharp, 
                            surface_points_flat,
                            surface_points_less_flat,
                            filtered_point_cloud);
  }
}
// Register the function as a benchmark
BENCHMARK(BM_ScanRegistration)->DenseRange(0, 2, 1);

BENCHMARK_MAIN();