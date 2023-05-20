cmake_minimum_required(VERSION 3.0)
project(mloam)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(Ceres REQUIRED)
find_package(GTest REQUIRED)
find_package(benchmark REQUIRED)

include_directories(include
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${GTEST_INCLUDE_DIR})

add_library(mloam
        src/mloam.cpp
        src/scan_registration.cpp
        src/odometry.cpp
        src/mapping.cpp)
target_link_libraries(mloam pcl_common pcl_io pcl_filters ${CERES_LIBRARIES})
target_include_directories(mloam PUBLIC include)

add_executable(my_loam main.cpp)
target_link_libraries(my_loam mloam)

add_executable(mloam_unit_tests 
        tests/main.cpp
        tests/odometry_tests.cpp
        tests/scan_registration_tests.cpp)
target_link_libraries(mloam_unit_tests GTest::GTest GTest::Main mloam)

add_custom_command(
        TARGET mloam_unit_tests
        COMMENT "Run tests"
        POST_BUILD
        COMMAND mloam_unit_tests)
