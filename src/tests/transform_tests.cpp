#include <gtest/gtest.h>
#include <transform.h>

using namespace AprilTagTracker;

TEST(AprilTagTrackerTransformTests, ConstructorAndGetters)
{
  tf2::Stamped<tf2::Transform> tag_tf;
  tag_tf.setOrigin(tf2::Vector3(1.0, 2.0, 3.0));
  tf2::Quaternion q;
  q.setRPY(0.0, M_PI_2, 0.0);
  tag_tf.setRotation(q);

  tf2::Stamped<tf2::Transform> servo_tf;
  servo_tf.setOrigin(tf2::Vector3(4.0, 5.0, 6.0));
  q.setRPY(0.0, 0.0, 0.0);
  servo_tf.setRotation(q);

  TagDetection detection;

  Transform test(detection, tag_tf, servo_tf);

  ASSERT_NEAR(1.0, test.getTagTf().getOrigin().getX(), 1e-10);
  ASSERT_NEAR(2.0, test.getTagTf().getOrigin().getY(), 1e-10);
  ASSERT_NEAR(3.0, test.getTagTf().getOrigin().getZ(), 1e-10);
  ASSERT_NEAR(4.0, test.getServoTf().getOrigin().getX(), 1e-10);
  ASSERT_NEAR(5.0, test.getServoTf().getOrigin().getY(), 1e-10);
  ASSERT_NEAR(6.0, test.getServoTf().getOrigin().getZ(), 1e-10);
  ASSERT_NEAR(M_PI_2, test.getTagTheta(), 1e-10);
}

TEST(AprilTagTrackerTransformTests, LessThanOperator)
{
  tf2::Stamped<tf2::Transform> servo_tf;
  TagDetection detection;

  tf2::Stamped<tf2::Transform> tag_tf1;
  tf2::Quaternion q;
  q.setRPY(0.0, M_PI_4, 0.0);
  tag_tf1.setRotation(q);
  Transform test1(detection, tag_tf1, servo_tf);

  tf2::Stamped<tf2::Transform> tag_tf2;
  q.setRPY(0.0, M_PI_4 * 1.1, 0.0);
  tag_tf2.setRotation(q);
  Transform test2(detection, tag_tf2, servo_tf);

  ASSERT_NEAR(M_PI_4, test1.getTagTheta(), 1e-10);
  ASSERT_NEAR(M_PI_4 * 1.1, test2.getTagTheta(), 1e-10);

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