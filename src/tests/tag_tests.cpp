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

TEST(AprilTagTrackerTagTests, AddTransform)
{
  Tag test(1, 1, 1.0);

  tf2::Stamped<tf2::Transform> tf;
  tf.setOrigin(tf2::Vector3(1.4, 0.0, 0.0));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 1.2);
  tf.setRotation(q);
  test.addTransform(tf);

  std::vector<Transform> data = test.getTransforms();

  ASSERT_NEAR(1.4, data[0].getTagTf().getOrigin().getX(), 1e-10);
  ASSERT_NEAR(1.2, data[0].getTagTheta(), 1e-10);
}

TEST(AprilTagTrackerTagTests, Add10Transforms)
{
  Tag test(1, 1, 1.0);

  tf2::Stamped<tf2::Transform> tf;
  for (int i = 0; i < 10; i++)
  {
    tf.setOrigin(tf2::Vector3((double)i, 0.0, 0.0));
    test.addTransform(tf);
  }

  std::vector<Transform> data = test.getTransforms();

  ASSERT_EQ(10, data.size());
  ASSERT_EQ(10, test.getSeq());
  ASSERT_EQ(9, data[0].getTagTf().getOrigin().getX());
  ASSERT_EQ(8, data[1].getTagTf().getOrigin().getX());
  ASSERT_EQ(7, data[2].getTagTf().getOrigin().getX());
  ASSERT_EQ(6, data[3].getTagTf().getOrigin().getX());
  ASSERT_EQ(5, data[4].getTagTf().getOrigin().getX());
  ASSERT_EQ(4, data[5].getTagTf().getOrigin().getX());
  ASSERT_EQ(3, data[6].getTagTf().getOrigin().getX());
  ASSERT_EQ(2, data[7].getTagTf().getOrigin().getX());
  ASSERT_EQ(1, data[8].getTagTf().getOrigin().getX());
  ASSERT_EQ(0, data[9].getTagTf().getOrigin().getX());
}

TEST(AprilTagTrackerTagTests, Add20Transforms)
{
  Tag test(1, 1, 1.0);

  tf2::Stamped<tf2::Transform> tf;
  for (int i = 0; i < 20; i++)
  {
    tf.setOrigin(tf2::Vector3((double)i, 0.0, 0.0));
    test.addTransform(tf);
  }

  std::vector<Transform> data = test.getTransforms();

  ASSERT_EQ(10, data.size());
  ASSERT_EQ(20, test.getSeq());
  ASSERT_EQ(19, data[0].getTagTf().getOrigin().getX());
  ASSERT_EQ(18, data[1].getTagTf().getOrigin().getX());
  ASSERT_EQ(17, data[2].getTagTf().getOrigin().getX());
  ASSERT_EQ(16, data[3].getTagTf().getOrigin().getX());
  ASSERT_EQ(15, data[4].getTagTf().getOrigin().getX());
  ASSERT_EQ(14, data[5].getTagTf().getOrigin().getX());
  ASSERT_EQ(13, data[6].getTagTf().getOrigin().getX());
  ASSERT_EQ(12, data[7].getTagTf().getOrigin().getX());
  ASSERT_EQ(11, data[8].getTagTf().getOrigin().getX());
  ASSERT_EQ(10, data[9].getTagTf().getOrigin().getX());
}

TEST(AprilTagTrackerTagTests, GetMedianFilteredTransformPartialFill)
{
  Tag test(1, 1, 1.0);

  tf2::Stamped<tf2::Transform> tf;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.5);
  tf.setRotation(q);
  test.addTransform(tf);
  q.setRPY(0.0, 0.0, 1.2);
  tf.setRotation(q);
  test.addTransform(tf);
  q.setRPY(0.0, 0.0, 0.2);
  tf.setRotation(q);
  test.addTransform(tf);

  // Check that it's the middle value
  ASSERT_EQ(3, test.getSeq());
  ASSERT_NEAR(0.5, test.getMedianFilteredTransform().getTagTheta(), 1e-10);

  // Check that the transform order hasn't been altered
  std::vector<Transform> data = test.getTransforms();
  ASSERT_EQ(3, data.size());
  ASSERT_NEAR(0.5, data[2].getTagTheta(), 1e-10);
  ASSERT_NEAR(1.2, data[1].getTagTheta(), 1e-10);
  ASSERT_NEAR(0.2, data[0].getTagTheta(), 1e-10);

}

TEST(AprilTagTrackerTagTests, GetMedianFilteredTransformEmpty)
{
  Tag test(1, 1, 1.0);

  // Check that it's the middle value
  ASSERT_EQ(0, test.getTransforms().size());
  ASSERT_NEAR(0.0, test.getMedianFilteredTransform().getTagTf().getOrigin().getX(), 1e-10);
}

TEST(AprilTagTrackerTagTests, GetMedianFilteredTransform)
{
  Tag test(1, 1, 1.0);

  tf2::Stamped<tf2::Transform> tf;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.5);
  tf.setRotation(q);
  test.addTransform(tf);
  q.setRPY(0.0, 0.0, 1.2);
  tf.setRotation(q);
  test.addTransform(tf);
  q.setRPY(0.0, 0.0, 0.2);
  tf.setRotation(q);
  test.addTransform(tf);
  q.setRPY(0.0, 0.0, 0.5);
  tf.setRotation(q);
  test.addTransform(tf);
  q.setRPY(0.0, 0.0, 1.2);
  tf.setRotation(q);
  test.addTransform(tf);
  q.setRPY(0.0, 0.0, 0.2);
  tf.setRotation(q);
  test.addTransform(tf);
  q.setRPY(0.0, 0.0, 0.5);
  tf.setRotation(q);
  test.addTransform(tf);
  q.setRPY(0.0, 0.0, 1.2);
  tf.setRotation(q);
  test.addTransform(tf);
  q.setRPY(0.0, 0.0, 0.2);
  tf.setRotation(q);
  test.addTransform(tf);
  q.setRPY(0.0, 0.0, 0.5);
  tf.setRotation(q);
  test.addTransform(tf);
  q.setRPY(0.0, 0.0, 1.2);
  tf.setRotation(q);
  test.addTransform(tf);
  q.setRPY(0.0, 0.0, 0.2);
  tf.setRotation(q);
  test.addTransform(tf);
  q.setRPY(0.0, 0.0, 0.5);
  tf.setRotation(q);
  test.addTransform(tf);
  q.setRPY(0.0, 0.0, 1.2);
  tf.setRotation(q);
  test.addTransform(tf);
  q.setRPY(0.0, 0.0, 0.2);
  tf.setRotation(q);
  test.addTransform(tf);

  std::vector<Transform> data = test.getTransforms();

  ASSERT_EQ(10, data.size());
  ASSERT_EQ(15, test.getSeq());
  ASSERT_NEAR(0.5, test.getMedianFilteredTransform().getTagTheta(), 1e-10);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}