#include <string>
#include "mloam/scan_registration.h"
#include "glad/glad.h"
#include "GLFW/glfw3.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "visualizer/camera3d.h"
#include "visualizer/gl.h"

Camera3d cam;
GLFWwindow* win;

/* We keep a record of the change of mouse position (to move the camera around), so
 * we need to register if its the first time we got a mouse position */
double prev_mouse_x = 0.0;
double prev_mouse_y = 0.0;
bool mouse_left_pushed = false;

void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods);

pcl::PointCloud<pcl::PointXYZI> point_cloud;

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
  static bool show_original_pt_cloud = true;
  ImGui::Checkbox("Show Original Point Cloud", &show_original_pt_cloud);
  
  static ImVec4 lidar_pt_color = ImVec4(1.0, 0.0, 0.0, 1.0);
  ImGui::ColorPicker4("Original Lidar Point Cloud Color", &lidar_pt_color.x);
  static int point_size = 5;
  ImGui::InputInt("Point Size", &point_size, 1, 1);

  if (show_original_pt_cloud) {
    // LidarViewer_Draw(point_cloud, &cam, lidar_pt_color, point_size);
  }
  ImGui::End();

}

int main(int argc, char **argv) {
  
  cam.SetWindowSize(800, 600);
  cam.SetCameraPosition(Eigen::Vector3f(0, 50, 0));
  win = GL_CreateWindow("MLOAM", 800, 600);
  glfwSetKeyCallback(win, key_callback);
  glfwSetScrollCallback(win, scroll_callback);
  glfwSetMouseButtonCallback(win, mouse_button_callback);
  // Start with the the first (0) binary lidar file
  LoadLidarData(0);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void) io;

  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(win, true);
  ImGui_ImplOpenGL3_Init("#version 150");

  ScanRegistration_Run(point_cloud);
  while (!glfwWindowShouldClose(win)) {
    glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    glfwPollEvents();

    if (!ImGui::GetIO().WantCaptureMouse) {
      double x, y;
      glfwGetCursorPos(win, &x, &y);
      mouse_callback(win, x, y);
    }
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    MainMenuWindow();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(win);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  return 0;
}

void mouse_callback(GLFWwindow *win, double xpos, double ypos) {

  static const double angle_scale = 0.1;
  if (mouse_left_pushed) {
    double delta_x = (xpos - prev_mouse_x) * angle_scale;
    double delta_y = (ypos - prev_mouse_y) * angle_scale;
    prev_mouse_x = xpos;
    prev_mouse_y = ypos;
    /* As the mouse moves up down on the screen (which is a change of y position) we want to change
     * the pitch angle. Thats a rotation around the X axis in OpenGL.
     * Likewise, if the moves left/right (change of X position) we want to change the yaw angle. And thats
     * a rotation around Y axis in OpenGL */
    cam.RotateByIncrement(delta_y, -delta_x, 0.0);
  }
}

void key_callback(GLFWwindow *win, int key, int scancode, int action, int mods) {
  if (key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
  }
  if (key == GLFW_KEY_UP && action == GLFW_PRESS) {
    cam.target.z() += 5.0f;
    cam.SetTargetPosition(cam.target);
  }
  if (key == GLFW_KEY_DOWN && action == GLFW_PRESS) {
    cam.target.z() -= 5.0f;
    cam.SetTargetPosition(cam.target);
  }
  if (key == GLFW_KEY_LEFT && action == GLFW_PRESS) {
    cam.target.x() -= 5.0f;
    cam.SetTargetPosition(cam.target);
  }
  if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS) {
    cam.target.x() += 5.0f;
    cam.SetTargetPosition(cam.target);
  }
}

void scroll_callback(GLFWwindow *win, double xoffset, double yoffset) {
  cam.ChangeFieldOfView(yoffset * 0.1);
}

void mouse_button_callback(GLFWwindow *win, int button, int action, int mods) {
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    if (action == GLFW_PRESS) {
      /* Get the mouse position from when the left button is clicked */
      glfwGetCursorPos(win, &prev_mouse_x, &prev_mouse_y);
      mouse_left_pushed = true;
    }
    else {
      mouse_left_pushed = false;
    }
  }
}