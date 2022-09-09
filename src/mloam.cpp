#include "mloam/mloam.h"


pcl::PointCloud<pcl::PointXYZI> mloam::LoadKittiData(const std::string &filePath,
                                                     int binaryFileNum) {
  pcl::PointCloud<pcl::PointXYZI> point_cloud;

  /* For holding the /my/path/to/files/0000000000.bin */
  char full_file_path[512];
  int fd = -1;
  /* Create the full path.
     * The %010d.bin says -> Create a 10 wide string, and pad with 0
     * so "%010d.bin with bin_cnt == 1234 would create a string 0000001234.bin */
  sprintf(full_file_path, "%s%010d.bin", filePath.c_str(), binaryFileNum);

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
    return point_cloud;
  }
  num = fread(data, sizeof(float), num, stream)/4;
  for (int32_t i=0; i<num; i++) {
    point_cloud.push_back(pcl::PointXYZI(*px, *py, *pz, *pr));
    px+=4; py+=4; pz+=4; pr+=4;
  }
  fclose(stream);
  free(data);
  return point_cloud;
}