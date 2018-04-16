#include <gtest/gtest.h>
#include <pose_estimate_filter/pose_estimate_filter.h>

using namespace apriltag_tracker;

TEST(PoseEstimateFilterTests, TrivialTest)
{
  ASSERT_EQ(true, true);
}

TEST(PoseEstimateFilterTests, getRPY)
{
  tf2::Quaternion q;
  double roll, pitch, yaw;

  q.setRPY(0.0, 0.0, 0.0);
  PoseEstimateFilter::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,   0.0, 1e-10);
  ASSERT_NEAR(pitch,  0.0, 1e-10);
  ASSERT_NEAR(yaw,    0.0, 1e-10);

  q.setRPY(0.0, 0.0, 1.0);
  PoseEstimateFilter::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,   0.0, 1e-10);
  ASSERT_NEAR(pitch,  0.0, 1e-10);
  ASSERT_NEAR(yaw,    1.0, 1e-10);

  q.setRPY(0.0, 0.0, -1.0);
  PoseEstimateFilter::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,   0.0, 1e-10);
  ASSERT_NEAR(pitch,  0.0, 1e-10);
  ASSERT_NEAR(yaw,   -1.0, 1e-10);

  q.setRPY(0.0, 0.0, 3*M_PI);
  PoseEstimateFilter::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,   0.0, 1e-10);
  ASSERT_NEAR(pitch,  0.0, 1e-10);
  ASSERT_NEAR(yaw,   M_PI, 1e-10);

  q.setRPY(0.0, 0.0, -3*M_PI);
  PoseEstimateFilter::getRPY(q, roll, pitch, yaw);
  ASSERT_NEAR(roll,   0.0, 1e-10);
  ASSERT_NEAR(pitch,  0.0, 1e-10);
  ASSERT_NEAR(yaw,   -M_PI, 1e-10);
}

TEST(PoseEstimateFilterTests, getTheta)
{
  tf2::Quaternion q;
  geometry_msgs::Quaternion orientation;

  q.setRPY(0.0, 0.0, M_PI);
  orientation = tf2::toMsg(q);
  ASSERT_NEAR(PoseEstimateFilter::getTheta(orientation), M_PI, 1e-10);

  q.setRPY(0.0, 0.0, -M_PI);
  orientation = tf2::toMsg(q);
  ASSERT_NEAR(PoseEstimateFilter::getTheta(orientation), -M_PI, 1e-10);

  q.setRPY(0.0, 0.0, 0.0);
  orientation = tf2::toMsg(q);
  ASSERT_NEAR(PoseEstimateFilter::getTheta(orientation), 0.0, 1e-10);

  q.setRPY(0.0, 0.0, 2*M_PI);
  orientation = tf2::toMsg(q);
  ASSERT_NEAR(PoseEstimateFilter::getTheta(orientation), 0.0, 1e-10);
}

TEST(PoseEstimateFilterTests, getAverageTheta)
{
  std::vector<double> thetas;
  double theta;

  thetas.push_back(-1.0);
  thetas.push_back(1.0);
  thetas.push_back(-2.0);
  thetas.push_back(2.0);
  theta = PoseEstimateFilter::getAverageTheta(thetas);
  ASSERT_NEAR(theta, 0.0, 1e-10);

  thetas.clear();
  thetas.push_back(M_PI_2);
  thetas.push_back(0.0);
  theta = PoseEstimateFilter::getAverageTheta(thetas);
  ASSERT_NEAR(theta, M_PI_4, 1e-10);

  thetas.clear();
  thetas.push_back(M_PI);
  thetas.push_back(M_PI_2);
  theta = PoseEstimateFilter::getAverageTheta(thetas);
  ASSERT_NEAR(theta, 3*M_PI_4, 1e-10);

  thetas.clear();
  thetas.push_back(M_PI);
  thetas.push_back(-M_PI_2);
  theta = PoseEstimateFilter::getAverageTheta(thetas);
  ASSERT_NEAR(theta, -3*M_PI_4, 1e-10);

  thetas.clear();
  thetas.push_back(-M_PI_2);
  thetas.push_back(0.0);
  theta = PoseEstimateFilter::getAverageTheta(thetas);
  ASSERT_NEAR(theta, -M_PI_4, 1e-10);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}