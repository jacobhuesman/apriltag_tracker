#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>
#include <geometry_msgs/TransformStamped.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "transform_cache_test");
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  static tf2_ros::TransformBroadcaster br;


  bool transformed = false;
  do
  {
    transformed = tfBuffer.canTransform("map", "tag4", ros::Time(0), ros::Duration(1));
    if (!transformed)
    {
      std::stringstream ss;
      ss << "Unable to find transform from " << "map" << " to " << "tag4";
      ROS_WARN("%s", ss.str().c_str());
    }
  } while(ros::ok() && !transformed);
  ROS_INFO("Found transform");

  transformed = false;
  do
  {
    transformed = tfBuffer.canTransform("base_link", "camera_optical", ros::Time(0), ros::Duration(1));
    if (!transformed)
    {
      std::stringstream ss;
      ss << "Unable to find transform from " << "base_link" << " to " << "camera_optical";
      ROS_WARN("%s", ss.str().c_str());
    }
  } while(ros::ok() && !transformed);
  ROS_INFO("Found tranform");

  geometry_msgs::TransformStamped transform_msg;

  tf2::Stamped<tf2::Transform> map_to_tag4_tf;
  transform_msg = tfBuffer.lookupTransform("map", "tag4", ros::Time(0));
  tf2::fromMsg(transform_msg, map_to_tag4_tf);

  tf2::Stamped<tf2::Transform> camera_optical_to_base_link_tf;
  transform_msg = tfBuffer.lookupTransform("camera_optical", "base_link", ros::Time(0));
  tf2::fromMsg(transform_msg, camera_optical_to_base_link_tf);

  transformed = false;
  do
  {
    transformed = tfBuffer.canTransform("camera_optical", "tag4_estimate", ros::Time(0), ros::Duration(1));
    if (!transformed)
    {
      std::stringstream ss;
      ss << "Unable to find transform from " << "camera_optical" << " to " << "base_link";
      ROS_WARN("%s", ss.str().c_str());
    }
  } while(ros::ok() && !transformed);
  ROS_INFO("Found tranform");

  // TODO add try catch blocks
  ros::Rate rate(30);
  while(ros::ok())
  {
    transform_msg = tfBuffer.lookupTransform("tag4_estimate", "camera_optical", ros::Time(0));
    tf2::Stamped<tf2::Transform> tag4_estimate_to_camera_optical_tf;
    tf2::fromMsg(transform_msg,tag4_estimate_to_camera_optical_tf);

    tf2::Transform pose_estimate = map_to_tag4_tf * tag4_estimate_to_camera_optical_tf * camera_optical_to_base_link_tf;
    tf2::Stamped<tf2::Transform> pose_estimate_stamped(pose_estimate, transform_msg.header.stamp, "map");
    geometry_msgs::TransformStamped pose_estimate_msg = tf2::toMsg(pose_estimate_stamped);
    pose_estimate_msg.child_frame_id = "base_link";

    br.sendTransform(pose_estimate_msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};
