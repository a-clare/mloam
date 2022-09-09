#include "gtest/gtest.h"
#include "scan_registration_tests.h"
#include "odometry_tests.h"

int main(int argc, char *argv[]) {
  // Set log level to max to supress all logs excpet errors during unit tests, just makes 
  // it easier to see tests passing/failing
  Log::LoggingLevel() = logging::LogLevel::Error;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}