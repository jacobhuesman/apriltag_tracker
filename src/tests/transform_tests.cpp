#include <gtest/gtest.h>
#include <transform.h>

using namespace AprilTagTracker;

TEST(AprilTagTrackerTransformTests, ConstructorAndGetters)
{
  tf2::Stamped<tf2::Transform> tf;
  tf.setOrigin(tf2::Vector3(1.0, 2.0, 3.0));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, M_PI_2);
  tf.setRotation(q);
  Transform test(tf);

  ASSERT_NEAR(1.0, test.getTagTf().getOrigin().getX(), 1e-10);
  ASSERT_NEAR(2.0, test.getTagTf().getOrigin().getY(), 1e-10);
  ASSERT_NEAR(3.0, test.getTagTf().getOrigin().getZ(), 1e-10);
  ASSERT_NEAR(M_PI_2, test.getTagTheta(), 1e-10);
}

TEST(AprilTagTrackerTransformTests, LessThanOperator)
{
  tf2::Stamped<tf2::Transform> tf1;
  tf2::Quaternion q1;
  q1.setRPY(0.0, 0.0, M_PI_2);
  tf1.setRotation(q1);
  Transform test1(tf1);

  tf2::Stamped<tf2::Transform> tf2;
  q1.setRPY(0.0, 0.0, M_PI_2 * 1.1);
  tf2.setRotation(q1);
  Transform test2(tf2);

  ASSERT_NEAR(M_PI_2, test1.getTagTheta(), 1e-10);
  ASSERT_NEAR(M_PI_2 * 1.1, test2.getTagTheta(), 1e-10);

  ASSERT_TRUE(test1 < test2);
  ASSERT_FALSE(test2 < test1);
  ASSERT_FALSE(test1 < test1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}