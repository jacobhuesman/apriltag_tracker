#include <gtest/gtest.h>
#include <tag.h>

using namespace AprilTagTracker;

TEST(AprilTagTrackerTagTests, Constructor)
{
Tag test(10, 8, 10.4);

ASSERT_EQ(10, test.getID());
//ASSERT_STREQ("tag10_estimate", test.getFrameID().c_str());
ASSERT_EQ(8, test.getPriority());
ASSERT_NEAR(10.4, test.getSize(), 1e-10);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
