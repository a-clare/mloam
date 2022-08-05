
#include "loam/lidar_viewer.h"
#include "visualizer/gl.h"
#include "visualizer/camera3d.h"
#include "imgui.h"
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/* Lidar vertex shader */
static const char *lidar_vs_src =
    "#version 330 core \n"
    "float colormap_red(float x) {\n"
    "    if (x < 0.7) {\n"
    "        return 4.0 * x - 1.5;\n"
    "    } else {\n"
    "        return -4.0 * x + 4.5;\n"
    "    }\n"
    "}\n"
    "\n"
    "float colormap_green(float x) {\n"
    "    if (x < 0.5) {\n"
    "        return 4.0 * x - 0.5;\n"
    "    } else {\n"
    "        return -4.0 * x + 3.5;\n"
    "    }\n"
    "}\n"
    "\n"
    "float colormap_blue(float x) {\n"
    "    if (x < 0.3) {\n"
    "       return 4.0 * x + 0.5;\n"
    "    } else {\n"
    "       return -4.0 * x + 2.5;\n"
    "    }\n"
    "}\n"
    "\n"
    "vec4 colormap(float x) {\n"
    "    float r = clamp(colormap_red(x), 0.0, 1.0);\n"
    "    float g = clamp(colormap_green(x), 0.0, 1.0);\n"
    "    float b = clamp(colormap_blue(x), 0.0, 1.0);\n"
    "    return vec4(r, g, b, 1.0);\n"
    "}\n"

    "layout (location = 0) in vec4 lidar_pt;\n"
    "uniform mat4 model;\n"
    "uniform mat4 view;\n"
    "uniform mat4 projection;\n"
    "uniform vec4 color;\n"
    "uniform int point_size;\n"
    "uniform bool use_fixed_color;\n"
    "out vec4 vertexColor;\n"
    "void main() {\n"
    "  gl_Position = projection * view * model * vec4(lidar_pt.x, lidar_pt.y, lidar_pt.z, 1.0f);\n"
    "  gl_PointSize = point_size;\n"
    "  if (use_fixed_color) {\n"
    "    vertexColor = color;\n"
    "  } else {"
    "    vertexColor = colormap(lidar_pt.w / 1.0);\n"
    "  }\n"
    "}\n\0";

/* Lidar fragment shader */
static const char *lidar_fs_src =
    "#version 330 core \n"
    "in vec4 vertexColor;\n"
    "out vec4 FragColor;\n"
    "void main() {\n"
    //        "   FragColor = vec4(0.0f, 1.0f, 1.0f, 1.0f);\n"
    "   FragColor = vertexColor;\n"
    "}\n\0";

/* The shader program used when drawing the lidar points as a single fixed color */
static GLuint lidar_shader_prog = 0;

/* So the coordinate system for OpenGL is:
 * +X is to the right of the screen
 * +Y is up on the screen
 * +Z is out of the screen towards you
 * The lidar data from the kitti data set is:
 * +X Out of the screen, away from you
 * +Y is to the right of the screen
 * +Z is up on the screen
 * So we need to rotate the lidar data for viewing. This is done through the model matrix.
 * Setting the data below is:
 * RotX(90 degrees) * RotZ(90 degrees) in homogeneous (so its 4x4)
 */
/* The model matrix for viewing the lidar data */
static const Mat4f model = (Mat4f){
    .mat = {0.0f, 0.0f, -1.0f, 0.0f,
            -1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f}};

void LidarViewer_Setup() {

  unsigned int vs, fs;
  vs = glCreateShader(GL_VERTEX_SHADER);
  glShaderSource(vs, 1, &lidar_vs_src, NULL);
  glCompileShader(vs);
  int success;
  char info_log[512];
  glGetShaderiv(vs, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(vs, 512, NULL, info_log);
    printf("ERROR: Vertex shader compilation failed %s\n", info_log);
    return;
  }

  fs = glCreateShader(GL_FRAGMENT_SHADER);
  glShaderSource(fs, 1, &lidar_fs_src, NULL);
  glCompileShader(fs);
  glGetShaderiv(fs, GL_COMPILE_STATUS, &success);
  if (!success) {
    glGetShaderInfoLog(fs, 512, NULL, info_log);
    printf("ERROR: Fragment shader compilation failed %s\n", info_log);
    return;
  }

  lidar_shader_prog = glCreateProgram();
  glAttachShader(lidar_shader_prog, vs);
  glAttachShader(lidar_shader_prog, fs);
  glLinkProgram(lidar_shader_prog);
  glGetProgramiv(lidar_shader_prog, GL_LINK_STATUS, &success);
  if (!success) {
    glGetProgramInfoLog(lidar_shader_prog, 512, NULL, info_log);
    printf("ERROR: Program shader compilation failed %s\n", info_log);
    return;
  }
  /* Dont need the shaders anymore once loaded onto the GPU */
  glDeleteShader(vs);
  glDeleteShader(fs);
}

void LidarViewer_Draw(const pcl::PointCloud<pcl::PointXYZI> &scan,
                      const Camera3d *cam,
                      const ImVec4 color,
                      int pointSize) {
  if (scan.size() == 0) {
    return;
  }
  struct pt {
    float x;
    float y;
    float z;
    float i;
  };
  std::vector<pt> pts;
  for (auto &point : scan) {
    pt npt;
    npt.x = point.x;
    npt.y = point.y;
    npt.z = point.z;
    npt.i = point.intensity;
    pts.push_back(npt);
  }
  unsigned int vbo;
  unsigned int vao;
  glGenVertexArrays(1, &vao);
  glGenBuffers(1, &vbo);
  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, pts.size() * sizeof(pcl::PointXYZI), &pts[0], GL_DYNAMIC_DRAW);
  glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  glBindVertexArray(0);

  glUseProgram(lidar_shader_prog);
  GLint model_loc = glGetUniformLocation(lidar_shader_prog, "model");
  glUniformMatrix4fv(model_loc, 1, GL_FALSE, &model.mat[0]);
  GLint view_loc = glGetUniformLocation(lidar_shader_prog, "view");
  glUniformMatrix4fv(view_loc, 1, GL_FALSE, &cam->view.mat[0]);
  GLint projection_loc = glGetUniformLocation(lidar_shader_prog, "projection");
  glUniformMatrix4fv(projection_loc, 1, GL_FALSE, &cam->projection.mat[0]);

  GLint color_loc = glGetUniformLocation(lidar_shader_prog, "color");
  glUniform4f(color_loc, color.x, color.y, color.z, color.w);
  GLint fixed_color = glGetUniformLocation(lidar_shader_prog, "use_fixed_color");
  glUniform1i(fixed_color, 1);
  GLint point_size = glGetUniformLocation(lidar_shader_prog, "point_size");
  glUniform1i(point_size, pointSize);

  glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
  glBindVertexArray(vao);
  glDrawArrays(GL_POINTS, 0, pts.size());
  glBindVertexArray(0);
}
