#include <gtest/gtest.h>
#include <apriltag_tracker/transform.h>

using namespace apriltag_tracker;

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

  tf2::Transform map_to_tag_tf;
  map_to_tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  q.setRPY(M_PI_2, 0.0, M_PI_2);
  map_to_tag_tf.setRotation(q);

  TagDetection detection;
  CompareType compare_mode = CompareType::theta;

  Transform test(detection, tag_tf, servo_tf, map_to_tag_tf, &compare_mode);

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

  tf2::Transform map_to_tag_tf;
  tf2::Quaternion q;
  q.setRPY(M_PI_2, 0.0, M_PI_2);
  map_to_tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  map_to_tag_tf.setRotation(q);


  tf2::Stamped<tf2::Transform> tag_tf1;
  q.setRPY(0.0, M_PI_4, 0.0);
  tag_tf1.setRotation(q);
  CompareType compare_mode = CompareType::theta;

  Transform test1(detection, tag_tf1, servo_tf, map_to_tag_tf, &compare_mode);

  tf2::Stamped<tf2::Transform> tag_tf2;
  q.setRPY(0.0, M_PI_4 * 1.1, 0.0);
  tag_tf2.setRotation(q);
  Transform test2(detection, tag_tf2, servo_tf, map_to_tag_tf, &compare_mode);

  ASSERT_NEAR(M_PI_4, test1.getTagTheta(), 1e-10);
  ASSERT_NEAR(M_PI_4 * 1.1, test2.getTagTheta(), 1e-10);

  ASSERT_TRUE(test1 < test2);
  ASSERT_FALSE(test2 < test1);
  ASSERT_FALSE(test1 < test1);
}

TEST(AprilTagTrackerTransformTests, GetRPY)
{
  tf2::Quaternion q;
  double roll, pitch, yaw;

  q.setRPY(0.0, 0.0, 0.0);
  Transform::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,  0.0, 1E-10);
  ASSERT_NEAR(pitch, 0.0, 1E-10);
  ASSERT_NEAR(yaw,   0.0, 1E-10);

  q.setRPY(M_PI_2, 0.0, 0.0);
  Transform::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,   M_PI_2, 1E-10);
  ASSERT_NEAR(pitch,     0.0, 1E-10);
  ASSERT_NEAR(yaw,       0.0, 1E-10);

  q.setRPY(-M_PI_2, 0.0, 0.0);
  Transform::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,  -M_PI_2, 1E-10);
  ASSERT_NEAR(pitch,     0.0, 1E-10);
  ASSERT_NEAR(yaw,       0.0, 1E-10);

  q.setRPY(0.0, M_PI_2, 0.0);
  Transform::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,      0.0, 1E-10);
  ASSERT_NEAR(pitch,  M_PI_2, 1E-10);
  ASSERT_NEAR(yaw,       0.0, 1E-10);

  q.setRPY(0.0, -M_PI_2, 0.0);
  Transform::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,      0.0, 1E-10);
  ASSERT_NEAR(pitch, -M_PI_2, 1E-10);
  ASSERT_NEAR(yaw,       0.0, 1E-10);

  q.setRPY(0.0, 0.0, M_PI_2);
  Transform::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,      0.0, 1E-10);
  ASSERT_NEAR(pitch,     0.0, 1E-10);
  ASSERT_NEAR(yaw,    M_PI_2, 1E-10);

  q.setRPY(0.0, 0.0, -M_PI_2);
  Transform::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,      0.0, 1E-10);
  ASSERT_NEAR(pitch,     0.0, 1E-10);
  ASSERT_NEAR(yaw,   -M_PI_2, 1E-10);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}