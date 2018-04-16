#ifndef POSE_ESTIMATE_FILTER_H
#define POSE_ESTIMATE_FILTER_H

#include <geometry_msgs/PoseStamped.h>
#include <boost/thread/thread.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>

using geometry_msgs::PoseStamped;

namespace apriltag_tracker
{

class PoseEstimateFilter
{
public:
  PoseEstimateFilter(int list_size, double max_dt);
  void addPoseEstimate(PoseStamped &pose);
  PoseStamped getMovingAverageTransform();
  PoseStamped getMovingAverageTransform(ros::Time current_time);
  void flushOldPoses(std::list<PoseStamped> *poses, ros::Time current_time);

  static void getRPY(tf2::Quaternion q, double &roll, double &pitch, double &yaw);
  static double getTheta(geometry_msgs::Quaternion orientation);
  static double getAverageTheta(std::vector<double> thetas);

private:
  boost::mutex *mutex;
  int list_size;
  int seq;
  std::list<PoseStamped> poses;
  ros::Duration max_dt;
};

}

#endif //POSE_ESTIMATE_FILTER_H
