#include <gtest/gtest.h>
#include <apriltag_tracker/tag.h>

using namespace apriltag_tracker;

TEST(AprilTagTrackerTagTests, Constructor)
{
  Tag test(10, 8, 10.4);

  ASSERT_EQ(10, test.getID());
  //ASSERT_STREQ("tag10_estimate", test.getFrameID().c_str());
  ASSERT_EQ(8, test.getPriority());
  ASSERT_NEAR(10.4, test.getSize(), 1e-10);
}

TEST(AprilTagTrackerTagTests, AddTransform)
{
  Tag test(1, 1, 1.0);

  tf2::Stamped<tf2::Transform> tag_tf;
  tag_tf.setOrigin(tf2::Vector3(1.4, 0.0, 0.0));
  tf2::Quaternion q;
  q.setRPY(0.0, 1.2, 0.0);
  tag_tf.setRotation(q);

  tf2::Stamped<tf2::Transform> servo_tf;
  servo_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  q.setRPY(0.0, 0.0, 0.0);
  servo_tf.setRotation(q);

  TagDetection detection;

  test.addTransform(detection, tag_tf, servo_tf, 0);

  std::vector<Transform> data = test.getTransforms();

  ASSERT_NEAR(1.4, data[0].getTagTf().getOrigin().getX(), 1e-10);
  ASSERT_NEAR(1.2, data[0].getTagTheta(), 1e-10);
}

TEST(AprilTagTrackerTagTests, Add5Transforms)
{
  Tag test(1, 1, 1.0);

  tf2::Stamped<tf2::Transform> servo_tf;
  TagDetection detection;
  tf2::Stamped<tf2::Transform> tag_tf;
  for (int i = 0; i < 5; i++)
  {
    tag_tf.setOrigin(tf2::Vector3((double)i, 0.0, 0.0));
    test.addTransform(detection, tag_tf, servo_tf, 0);
  }

  std::vector<Transform> data = test.getTransforms();

  ASSERT_EQ(5, data.size());
  ASSERT_EQ(4, data[0].getTagTf().getOrigin().getX());
  ASSERT_EQ(3, data[1].getTagTf().getOrigin().getX());
  ASSERT_EQ(2, data[2].getTagTf().getOrigin().getX());
  ASSERT_EQ(1, data[3].getTagTf().getOrigin().getX());
  ASSERT_EQ(0, data[4].getTagTf().getOrigin().getX());
}

TEST(AprilTagTrackerTagTests, Add20Transforms)
{
  Tag test(1, 1, 1.0);

  tf2::Stamped<tf2::Transform> servo_tf;
  TagDetection detection;
  tf2::Stamped<tf2::Transform> tag_tf;
  for (unsigned int i = 0; i < 20; i++)
  {
    tag_tf.setOrigin(tf2::Vector3((double)i, 0.0, 0.0));
    test.addTransform(detection, tag_tf, servo_tf, i + 1);
  }

  std::vector<Transform> data = test.getTransforms();

  ASSERT_EQ(20, data.size());
  ASSERT_EQ(20, test.getSeq());
  ASSERT_EQ(19, data[0].getTagTf().getOrigin().getX());
  ASSERT_EQ(18, data[1].getTagTf().getOrigin().getX());
  ASSERT_EQ(17, data[2].getTagTf().getOrigin().getX());
  ASSERT_EQ(16, data[3].getTagTf().getOrigin().getX());
  ASSERT_EQ(15, data[4].getTagTf().getOrigin().getX());
}

TEST(AprilTagTrackerTagTests, GetMedianFilteredTransformEmpty)
{
  Tag test(1, 1, 1.0);

  // Check that it's the middle value
  ASSERT_EQ(0, test.getTransforms().size());
  ASSERT_THROW(test.getMedianFilteredTransform(), unable_to_find_transform_error);
}

TEST(AprilTagTrackerTagTests, GetMedianFilteredTransformPartialFill)
{
  Tag test(1, 1, 1.0);

  tf2::Stamped<tf2::Transform> servo_tf;
  TagDetection detection;
  tf2::Stamped<tf2::Transform> tag_tf;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.5, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);
  q.setRPY(0.0, 1.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);
  q.setRPY(0.0, 0.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);

  ASSERT_THROW(test.getMedianFilteredTransform(), unable_to_find_transform_error);

  // Check that the transform order hasn't been altered
  std::vector<Transform> data = test.getTransforms();
  ASSERT_EQ(3, data.size());
  ASSERT_NEAR(0.5, data[2].getTagTheta(), 1e-10);
  ASSERT_NEAR(1.2, data[1].getTagTheta(), 1e-10);
  ASSERT_NEAR(0.2, data[0].getTagTheta(), 1e-10);

}

TEST(AprilTagTrackerTagTests, GetMedianFilteredTransform)
{
  Tag test(1, 1, 1.0);

  tf2::Stamped<tf2::Transform> servo_tf;
  TagDetection detection;
  tf2::Stamped<tf2::Transform> tag_tf;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.5, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf,  1);
  q.setRPY(0.0, 1.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf,  2);
  q.setRPY(0.0, 0.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf,  3);
  q.setRPY(0.0, 0.5, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf,  4);
  q.setRPY(0.0, 1.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf,  5);
  q.setRPY(0.0, 0.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf,  6);
  q.setRPY(0.0, 0.5, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf,  7);
  q.setRPY(0.0, 1.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf,  8);
  q.setRPY(0.0, 0.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf,  9);
  q.setRPY(0.0, 0.5, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 10);
  q.setRPY(0.0, 1.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 11);
  q.setRPY(0.0, 0.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 12);
  q.setRPY(0.0, 0.5, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 13);
  q.setRPY(0.0, 1.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 14);
  q.setRPY(0.0, 0.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 15);

  std::vector<Transform> data = test.getTransforms();

  ASSERT_EQ(15, data.size());
  ASSERT_EQ(15, test.getSeq());
  ASSERT_NEAR(0.5, test.getMedianFilteredTransform().getTagTheta(), 1e-10);
}

TEST(AprilTagTrackerTagTests, GetMovingAverageTransform)
{
  Tag test(1, 1, 1.0);
  ros::Time::init();

  tf2::Stamped<tf2::Transform> servo_tf;
  TagDetection detection;
  tf2::Stamped<tf2::Transform> tag_tf;
  tag_tf.stamp_ = ros::Time::now();
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  1);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  2);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  3);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  4);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  5);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  6);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  7);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  8);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  9);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 10);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 11);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 12);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 13);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 14);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 15);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 16);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 17);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 18);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 19);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 20);

  std::vector<Transform> data = test.getTransforms();

  ASSERT_EQ(20, data.size());
  ASSERT_EQ(20, test.getSeq());
  ASSERT_NEAR(4.0/20.0, test.getMovingAverageTransform().getTagTf().getOrigin().getX(), 1e-10);
}

TEST(AprilTagTrackerTagTests, GetMovingAverageTransform5)
{
  Tag test(1, 1, 1.0);
  ros::Time::init();

  tf2::Stamped<tf2::Transform> servo_tf;
  TagDetection detection;
  tf2::Stamped<tf2::Transform> tag_tf;
  tag_tf.stamp_ = ros::Time::now();
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  1);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  2);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  3);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  4);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  5);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  6);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  7);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  8);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  9);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 10);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 11);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 12);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 13);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 14);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 15);

  std::vector<Transform> data = test.getTransforms();

  ASSERT_EQ(15, data.size());
  ASSERT_EQ(15, test.getSeq());
  ASSERT_NEAR(2.0/5.0, test.getMovingAverageTransform(5).getTagTf().getOrigin().getX(), 1e-10);
}

TEST(AprilTagTrackerTagTests, GetAngleFromCenterMostRecent)
{
  Tag test(1, 1, 1.0);
  ros::Time::init();

  tf2::Stamped<tf2::Transform> servo_tf;
  TagDetection detection;
  tf2::Stamped<tf2::Transform> tag_tf;
  tag_tf.stamp_ = ros::Time::now();
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf,  1);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf,  2);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf,  3);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf,  4);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf,  5);

  std::vector<Transform> data = test.getTransforms();

  ASSERT_NEAR(M_PI_4, test.getAngleFromCenter(1), 1e-10);
}

TEST(AprilTagTrackerTagTests, GetAngleFromCenterFiltered)
{
  Tag test(1, 1, 1.0);
  ros::Time::init();

  tf2::Stamped<tf2::Transform> servo_tf;
  TagDetection detection;
  tf2::Stamped<tf2::Transform> tag_tf;
  tag_tf.stamp_ = ros::Time::now();
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf, 1);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf, 2);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf, 3);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf, 4);

  ASSERT_NEAR(M_PI_4, test.getAngleFromCenter(4), 1e-10);

  tag_tf.setOrigin(tf2::Vector3(-1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf, 5);
  tag_tf.setOrigin(tf2::Vector3(-1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf, 6);
  tag_tf.setOrigin(tf2::Vector3( 1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf,  7);
  tag_tf.setOrigin(tf2::Vector3( 1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf,  8);

  ASSERT_NEAR(0.0, test.getAngleFromCenter(4), 1e-10);

  tag_tf.setOrigin(tf2::Vector3(-1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf, 5);
  tag_tf.setOrigin(tf2::Vector3(-1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf, 6);
  tag_tf.setOrigin(tf2::Vector3(-1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf, 7);
  tag_tf.setOrigin(tf2::Vector3(-1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf, 8);

  ASSERT_NEAR(-M_PI_4, test.getAngleFromCenter(4), 1e-10);
}

TEST(AprilTagTrackerTagTests, GetAngleFromCenterExceptionWhenUnloaded)
{
  Tag test(1, 1, 1.0);
  ros::Time::init();

  tf2::Stamped<tf2::Transform> servo_tf;
  TagDetection detection;
  tf2::Stamped<tf2::Transform> tag_tf;
  tag_tf.stamp_ = ros::Time::now();
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf,  1);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf,  2);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf,  3);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf,  4);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf,  5);

  std::vector<Transform> data = test.getTransforms();

  ASSERT_THROW(test.getAngleFromCenter(10), unable_to_find_transform_error);
}


TEST(AprilTagTrackerTagTests, GetAngleFromCenterOrdering)
{
  Tag test(1, 1, 1.0);
  ros::Time::init();

  tf2::Stamped<tf2::Transform> servo_tf;
  TagDetection detection;
  tf2::Stamped<tf2::Transform> tag_tf;
  tag_tf.stamp_ = ros::Time::now();
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  1);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  2);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  3);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  4);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  5);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  6);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  7);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  8);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf,  9);
  tag_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0)); test.addTransform(detection, tag_tf, servo_tf, 10);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf, 11);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf, 12);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf, 13);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf, 14);
  tag_tf.setOrigin(tf2::Vector3(1.0, 0.0, 1.0)); test.addTransform(detection, tag_tf, servo_tf, 15);

  std::vector<Transform> data = test.getTransforms();

  ASSERT_NEAR(M_PI_4, test.getAngleFromCenter(5), 1e-10);
}


TEST(AprilTagTrackerTagTests, GetMostRecentTransformEmpty)
{
  Tag test(1, 1, 1.0);

  ASSERT_THROW(test.getMostRecentTransform(), unable_to_find_transform_error);
}

TEST(AprilTagTrackerTagTests, GetMostRecentTransform1)
{
  Tag test(1, 1, 1.0);

  tf2::Stamped<tf2::Transform> servo_tf;
  tf2::Stamped<tf2::Transform> tag_tf;
  tf2::Quaternion q;
  TagDetection detection;
  q.setRPY(0.0, 0.5, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);

  ASSERT_NEAR(test.getMostRecentTransform().getTagTheta(), 0.5, 1e-10);
}

TEST(AprilTagTrackerTagTests, GetMostRecentTransform10)
{
  Tag test(1, 1, 1.0);

  tf2::Stamped<tf2::Transform> servo_tf;
  TagDetection detection;
  tf2::Stamped<tf2::Transform> tag_tf;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.5, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);
  q.setRPY(0.0, 1.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);
  q.setRPY(0.0, 0.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);
  q.setRPY(0.0, 0.5, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);
  q.setRPY(0.0, 1.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);
  q.setRPY(0.0, 0.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);
  q.setRPY(0.0, 0.5, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);
  q.setRPY(0.0, 1.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);
  q.setRPY(0.0, 0.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);
  q.setRPY(0.0, 0.5, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);
  q.setRPY(0.0, 1.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);
  q.setRPY(0.0, 0.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);
  q.setRPY(0.0, 0.5, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);
  q.setRPY(0.0, 1.2, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);
  q.setRPY(0.0, 0.1, 0.0); tag_tf.setRotation(q); test.addTransform(detection, tag_tf, servo_tf, 0);

  ASSERT_NEAR(test.getMostRecentTransform().getTagTheta(), 0.1, 1e-10);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}