cmake_minimum_required(VERSION 3.0)
project(mloam)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

include_directories(include
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR})

add_subdirectory(../visualizer ${CMAKE_CURRENT_BINARY_DIR}/visualizer)

include_directories(../imgui/)
include_directories(../imgui/backends)
add_library(imgui
        ../imgui/imgui.cpp
        ../imgui/imgui_demo.cpp
        ../imgui/imgui_draw.cpp
        ../imgui/imgui_tables.cpp
        ../imgui/imgui_widgets.cpp
        ../imgui/backends/imgui_impl_glfw.cpp
        ../imgui/backends/imgui_impl_opengl3.cpp)
target_link_libraries(imgui glfw glad)
target_include_directories(imgui PUBLIC ../imgui/backends)

add_library(mloam
        src/scan_registration.cpp
        src/lidar_viewer.cpp)
target_link_libraries(mloam ${PCL_LIBRARIES} visualizer)
target_include_directories(mloam PUBLIC include)

add_executable(my_loam main.cpp)
target_link_libraries(my_loam mloam imgui)