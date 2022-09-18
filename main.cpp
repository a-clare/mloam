#include <string>
#include "mloam/mloam.h"

pcl::PointCloud<pcl::PointXYZI> point_cloud;

// For scan registration
pcl::PointCloud<pcl::PointXYZI> corner_points_sharp;
pcl::PointCloud<pcl::PointXYZI> corner_points_less_sharp;
pcl::PointCloud<pcl::PointXYZI> surface_points_flat;
pcl::PointCloud<pcl::PointXYZI> surface_points_less_flat;
pcl::PointCloud<pcl::PointXYZI> filtered_point_cloud;

pcl::PointCloud<pcl::PointXYZI> laser_map;

bool LoadLidarData(int binaryFileNumber) {
  point_cloud.clear();
  corner_points_sharp.clear();
  surface_points_flat.clear();
  surface_points_less_flat.clear();
  filtered_point_cloud.clear();

  LOG_INFO << "Reading binary file number " << binaryFileNumber;
  std::string path_to_lidar_data = "/Users/adamclare/data/2011_09_30/2011_09_30_drive_0028_sync/velodyne_points/data/";
  bool success = false;
  /* For holding the /my/path/to/files/0000000000.bin */
  char full_file_path[512];
  int fd = -1;
  /* Create the full path.
     * The %010d.bin says -> Create a 10 wide string, and pad with 0
     * so "%010d.bin with bin_cnt == 1234 would create a string 0000001234.bin */
  sprintf(full_file_path, "%s%010d.bin", path_to_lidar_data.c_str(), binaryFileNumber);

  int32_t num = 1000000;
  float *data = (float*)malloc(num*sizeof(float));
  // pointers
  float *px = data+0;
  float *py = data+1;
  float *pz = data+2;
  float *pr = data+3;

  FILE *stream;
  stream = fopen (full_file_path,"rb");
  if (stream == NULL) {
    return false;
  }
  num = fread(data, sizeof(float), num, stream)/4;
  for (int32_t i=0; i<num; i++) {
    point_cloud.push_back(pcl::PointXYZI(*px, *py, *pz, *pr));
    px+=4; py+=4; pz+=4; pr+=4;
  }
  fclose(stream);
  free(data);
  return true;
}

int main(int argc, char **argv) {
   
  // Setting log level to error so it supresses all the logging info in ScanRegistration, for now as I test odometry 
  Log::LoggingLevel() = logging::LogLevel::Error;
  int binary_file_number = 1;
  while(LoadLidarData(binary_file_number)) {
    mloam::ScanRegistration(point_cloud, 
                            corner_points_sharp, 
                            corner_points_less_sharp, 
                            surface_points_flat,
                            surface_points_less_flat,
                            filtered_point_cloud);

    mloam::OdometryData odometry_data;
    mloam::Odometry(corner_points_sharp,
                    corner_points_less_sharp,
                    surface_points_flat,
                    surface_points_less_flat,
                    filtered_point_cloud,
                    odometry_data);

    std::cout << "1," << odometry_data.pose.pose.position.x() << "," 
                   << odometry_data.pose.pose.position.y() << "," 
                   << odometry_data.pose.pose.position.z() << std::endl;
    mloam::Mapping(corner_points_less_sharp,
                   surface_points_less_flat,
                   filtered_point_cloud,
                   odometry_data);

    binary_file_number += 1;
  }

  return 0;
}