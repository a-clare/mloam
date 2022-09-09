#include "gtest/gtest.h"
#include "scan_registration_tests.h"
#include "odometry_tests.h"

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}