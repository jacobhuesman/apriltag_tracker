#include <gtest/gtest.h>
#include <apriltag_tracker.h>

using namespace AprilTagTracker;

TEST(AprilTagTrackerTests, TrivialTest)
{
  ASSERT_EQ(true, true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}