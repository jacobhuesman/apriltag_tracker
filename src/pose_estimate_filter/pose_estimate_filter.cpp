#include <pose_estimate_filter/pose_estimate_filter.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>


using geometry_msgs::Point;
using geometry_msgs::Quaternion;
using namespace apriltag_tracker;


PoseEstimateFilter::PoseEstimateFilter(int list_size, double max_dt)
{
  this->seq = 0;
  this->list_size = list_size;
  this->max_dt = ros::Duration(max_dt);
}

void PoseEstimateFilter::addPoseEstimate(PoseStamped &pose)
{
  mutex->lock();
  poses.emplace_front(pose);
  while (poses.size() > list_size)
  {
    poses.pop_back();
  }
  mutex->unlock();
}

void PoseEstimateFilter::flushOldPoses(std::list<PoseStamped> *poses, ros::Time current_time)
{
  auto it = poses->begin();
  int good_poses = 0;
  bool clean = false;
  for (int i = 0; i < poses->size(); i++, it++)
  {
    ros::Duration time_diff(current_time - it->header.stamp);
    if (time_diff > max_dt)
    {
      clean = true;
      good_poses = i;
      break;
    }
  }
  if (clean)
  {
    for (int i = 0; (i + good_poses) < poses->size(); i++)
    {
      poses->pop_back();
    }
  }
}

PoseStamped PoseEstimateFilter::getMovingAverageTransform()
{
  return getMovingAverageTransform(ros::Time::now());
}

void PoseEstimateFilter::getRPY(tf2::Quaternion q, double &roll, double &pitch, double &yaw)
{
  tf2::Matrix3x3 matrix;
  matrix.setRotation(q);
  matrix.getRPY(roll, pitch, yaw);
}

double PoseEstimateFilter::getTheta(geometry_msgs::Quaternion orientation)
{
  double roll, pitch, yaw;
  tf2::Quaternion q;
  tf2::fromMsg(orientation, q);
  getRPY(q, roll, pitch, yaw);
  return yaw;
}

// Assumes that the thetas are close enough not to negate each other
double PoseEstimateFilter::getAverageTheta(std::vector<double> thetas)
{
  double x = 0.0, y = 0.0;
  for (int i = 0; i < thetas.size(); i++)
  {
    x += cos(thetas[i]);
    y += sin(thetas[i]);
  }
  return atan2(y,x);
}

PoseStamped PoseEstimateFilter::getMovingAverageTransform(ros::Time current_time)
{
  mutex->lock();

  flushOldPoses(&poses, current_time);

  auto it = poses.begin();
  double x = 0, y = 0, z = 0, th = 0;
  for (int i = 0; i < poses.size(); i++, it++)
  {
    Point position = it->pose.position;
    x += position.x;
    y += position.y;
    z += position.z;

  }
  mutex->unlock();
}


