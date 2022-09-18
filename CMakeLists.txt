cmake_minimum_required(VERSION 3.0)
project(mloam)

set(CMAKE_CXX_STANDARD 14)

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
target_link_libraries(mloam ${PCL_LIBRARIES} ${CERES_LIBRARIES})
target_include_directories(mloam PUBLIC include)

add_executable(my_loam main.cpp)
target_link_libraries(my_loam mloam)

add_executable(mloam_unit_tests 
  tests/main.cpp)
target_link_libraries(mloam_unit_tests GTest::GTest GTest::Main mloam)

add_custom_command(
        TARGET mloam_unit_tests
        COMMENT "Run tests"
        POST_BUILD
        COMMAND mloam_unit_tests)

add_executable(mloam_benchmarks 
  benchmarks/scan_registration_benchmarks.cpp)
target_link_libraries(mloam_benchmarks mloam benchmark::benchmark benchmark::benchmark_main)  