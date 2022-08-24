#include <string>
#include "mloam/mloam.h"
#include "mloam/lidar_viewer.h"
#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "mgl/mgl.h"

mgl::Camera3D cam;


pcl::PointCloud<pcl::PointXYZI> point_cloud;

// For scan registration
pcl::PointCloud<pcl::PointXYZI> corner_points_sharp;
pcl::PointCloud<pcl::PointXYZI> corner_points_less_sharp;
pcl::PointCloud<pcl::PointXYZI> surface_points_flat;
pcl::PointCloud<pcl::PointXYZI> surface_points_less_flat;
pcl::PointCloud<pcl::PointXYZI> filtered_point_cloud;

void LoadLidarData(int binaryFileNumber) {
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
  num = fread(data, sizeof(float), num, stream)/4;
  for (int32_t i=0; i<num; i++) {
    point_cloud.push_back(pcl::PointXYZI(*px, *py, *pz, *pr));
    px+=4; py+=4; pz+=4; pr+=4;
  }
  fclose(stream);
  free(data);
}

void MainMenuWindow() {
   
  ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Always);
  ImGui::Begin("Main");
  ImGui::Text("Num Lidar Points %d", point_cloud.size());
  
  static ImVec4 lidar_pt_color = ImVec4(1.0, 0.0, 0.0, 1.0);
  static int point_size = 5;
  if (ImGui::CollapsingHeader("Original Point Cloud")) {
    static bool show_original_pt_cloud = true;
    ImGui::Checkbox("Show Original Point Cloud", &show_original_pt_cloud);
    if (show_original_pt_cloud) {
      mloam::LidarViewer_Draw(point_cloud, cam, lidar_pt_color, point_size);
    }
    ImGui::InputInt("Point Size", &point_size, 1, 1);
    ImGui::ColorPicker4("Original Lidar Point Cloud Color", &lidar_pt_color.x);
  }

  if (ImGui::CollapsingHeader("Corner Points Sharp")) {
    ImGui::PushID("Corner Points Sharp");
    static bool show = false;
    static ImVec4 color = ImVec4(1.0, 1.0, 0.0, 1.0);
    static int point_size = 5;

    ImGui::Checkbox("Show", &show);
    ImGui::InputInt("Point Size", &point_size, 1, 1);
    ImGui::ColorPicker4("Color", &color.x);
    if (show) {
      mloam::LidarViewer_Draw(corner_points_sharp, cam, color, point_size);
    }
    ImGui::PopID();
  }

  if (ImGui::CollapsingHeader("Corner Points Less Sharp")) {
    ImGui::PushID("Corner Points Less Sharp");
    static bool show = false;
    static ImVec4 color = ImVec4(1.0, 1.0, 0.0, 1.0);
    static int point_size = 5;
    ImGui::Checkbox("Show", &show);
    ImGui::InputInt("Point Size", &point_size, 1, 1);
    ImGui::ColorPicker4("Color", &color.x);
    if (show) {
      mloam::LidarViewer_Draw(corner_points_less_sharp, cam, color, point_size);
    }
    ImGui::PopID();
  }

  if (ImGui::CollapsingHeader("Surface Points Flat")) {
    ImGui::PushID("Surface Points Flat");
    static bool show = false;
    static ImVec4 color = ImVec4(1.0, 1.0, 0.0, 1.0);
    static int point_size = 5;
    ImGui::Checkbox("Show", &show);
    ImGui::InputInt("Point Size", &point_size, 1, 1);
    ImGui::ColorPicker4("Color", &color.x);
    if (show) {
      mloam::LidarViewer_Draw(surface_points_flat, cam, color, point_size);
    }
    ImGui::PopID();
  }

  if (ImGui::CollapsingHeader("Surface Points Less Flat")) {
    ImGui::PushID("Surface Points Less Flat");
    static bool show = false;
    static ImVec4 color = ImVec4(1.0, 1.0, 0.0, 1.0);
    static int point_size = 5;
    ImGui::Checkbox("Show", &show);
    ImGui::InputInt("Point Size", &point_size, 1, 1);
    ImGui::ColorPicker4("Color", &color.x);
    if (show) {
      mloam::LidarViewer_Draw(surface_points_less_flat, cam, color, point_size);
    }
    ImGui::PopID();
  }
  ImGui::End();

}

int main(int argc, char **argv) {
  
  mgl::InitializeWindow(800, 600, "MLOAM");
  
  cam.SetPosition(0, 50, 50);
  LoadLidarData(0);

  mloam::LidarViewer_Setup();
  IMGUI_CHECKVERSION();
  
  mloam::ScanRegistration(point_cloud, 
                          corner_points_sharp, 
                          corner_points_less_sharp, 
                          surface_points_flat,
                          surface_points_less_flat,
                          filtered_point_cloud);

  while (!mgl::ShouldWindowClose()) {
    mgl::BeginFrame();

    mgl::UpdateCameraFromUserInput(cam);

    ImGui::ShowDemoWindow(nullptr);
    MainMenuWindow();

    mgl::EndFrame();
  }

  // ImGui_ImplOpenGL3_Shutdown();
  // ImGui_ImplGlfw_Shutdown();
  // ImGui::DestroyContext();
  return 0;
}