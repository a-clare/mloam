cmake_minimum_required(VERSION 3.0)
project(mloam)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

include_directories(include
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR})

add_subdirectory(../mgl ${CMAKE_CURRENT_BINARY_DIR}/mgl)

add_library(mloam
        src/scan_registration.cpp
        src/lidar_viewer.cpp)
target_link_libraries(mloam ${PCL_LIBRARIES} mgl)
target_include_directories(mloam PUBLIC include)

add_executable(my_loam main.cpp)
target_link_libraries(my_loam mloam mgl)