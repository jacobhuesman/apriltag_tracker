#include <pose_estimate_filter/pose_estimate_filter.h>
#include <ros/ros.h>

using namespace apriltag_tracker;

ros::Publisher *pose_pub;
PoseEstimateFilter *filter;

void callback(const geometry_msgs::PoseStampedConstPtr &pose)
{
  filter->addPoseEstimate(*pose.get());
  try
  {
    pose_pub->publish(filter->getMovingAverageTransform());
  }
  catch (empty_list_error &e)
  {
    ROS_WARN("%s", e.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_estimate_filter");
  ros::NodeHandle nh;
  pose_pub = new ros::Publisher;
  *pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_estimate", 100);

  filter = new PoseEstimateFilter(4, 1.0);

  ros::Subscriber node0 = nh.subscribe("/node0/pose_estimate", 100, callback);
  ros::Subscriber node1 = nh.subscribe("/node1/pose_estimate", 100, callback);

  ros::spin();
}