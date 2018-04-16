#include <gtest/gtest.h>
#include <apriltag_tracker/apriltag_tracker.h>
#include <apriltag_tracker/camera_info.h>
#include <apriltag_tracker/errors.h>

using namespace apriltag_tracker;

TEST(AprilTagTrackerTests, TrivialTest)
{
  ASSERT_EQ(true, true);
}

/*AprilTagTracker buildAprilTagTracker()
{
  // Create properties object
  CameraInfo *properties = new CameraInfo;
  properties->image_seq = 0;

  // Create tags
  std::vector<Tag> *tag_info = new std::vector<Tag>;
  Tag tag1(1, 2, 0.42545);
  Tag tag2(2, 1, 0.42545);

  tag_info->push_back(tag1);
  tag_info->push_back(tag2);

  // Add tag transforms
  tf2::Transform map_to_tag01_tf, map_to_tag02_tf;
  tf2::Quaternion q;
  q.setRPY(M_PI_2, 0.0, M_PI_2);
  map_to_tag01_tf.setOrigin(tf2::Vector3(0.0, -0.5, 0.0));
  map_to_tag01_tf.setRotation(q);
  map_to_tag02_tf.setOrigin(tf2::Vector3(0.0,  0.5, 0.0));
  map_to_tag02_tf.setRotation(q);

  // Initialize transforms cache
  TransformsCache transforms;

  q.setRPY(-M_PI_2, 0, -M_PI_2);
  tf2::Transform servo_joint_to_optical_link_tf;
  servo_joint_to_optical_link_tf.setRotation(q);
  servo_joint_to_optical_link_tf.setOrigin(tf2::Vector3(9.0e-3, 0.0, 25.0e-3));
  transforms.camera_optical_to_camera_mount = servo_joint_to_optical_link_tf.inverse();

  q.setRPY(0.0, 0, M_PI);
  tf2::Transform base_link_to_servo_base_link_tf;
  base_link_to_servo_base_link_tf.setRotation(q);
  base_link_to_servo_base_link_tf.setOrigin(tf2::Vector3(-0.4191, 0.0, 0.0));
  transforms.servo_base_link_to_base_link = base_link_to_servo_base_link_tf.inverse();

  AprilTagTrackerConfig *tracker_config = new AprilTagTrackerConfig();

  return AprilTagTracker(properties, tag_info, tracker_config, transforms);
}

TEST(AprilTagTrackerTests, Constructor)
{
  AprilTagTracker tracker = buildAprilTagTracker();

  ASSERT_THROW(tracker.getTransform(), unable_to_find_transform_error);
}*/

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}