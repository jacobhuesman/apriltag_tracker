#ifndef POSE_ESTIMATE_FILTER_H
#define POSE_ESTIMATE_FILTER_H

#include <geometry_msgs/PoseStamped.h>
#include <boost/thread/thread.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>


namespace apriltag_tracker
{

class PoseEstimateFilter
{
public:
  using geometry_msgs::PoseStamped;
  using geometry_msgs::Point;
  using geometry_msgs::Quaternion

  PoseEstimateFilter(int list_size, double max_dt);
  void addPoseEstimate(PoseStamped &pose);
  PoseStamped getMovingAverageTransform();
  PoseStamped getMovingAverageTransform(ros::Time current_time);
  void flushOldPoses(std::list<PoseStamped> *poses, ros::Time current_time);

  static void getRPY(tf2::Quaternion q, double &roll, double &pitch, double &yaw);
  static double getTheta(Quaternion orientation);
  static Quaternion getAverageOrientation(std::list<PoseStamped> &poses);
  static Point getAveragePosition(std::list<PoseStamped> &poses);

private:
  boost::mutex *mutex;
  int list_size;
  unsigned int seq;
  std::list<PoseStamped> poses;
  ros::Duration max_dt;
};

}

#endif //POSE_ESTIMATE_FILTER_H
