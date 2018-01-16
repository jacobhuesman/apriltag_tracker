#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>



int main(int argc, char** argv){
  ros::init(argc, argv, "transform_cache_test");
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);


  bool transformed = false;
  do
  {
    transformed = tfBuffer.canTransform("map", "tag04", ros::Time::now(), ros::Duration(1));
    if (!transformed)
    {
      std::stringstream ss;
      ss << "Unable to find transform from " << "map" << " to " << "tag04";
      ROS_WARN("%s", ss.str().c_str());
    }
  } while(ros::ok() && !transformed);
  ROS_INFO("Found transform");

  transformed = false;
  do
  {
    transformed = tfBuffer.canTransform("base_link", "camera_optical", ros::Time::now(), ros::Duration(1));
    if (!transformed)
    {
      std::stringstream ss;
      ss << "Unable to find transform from " << "base_link" << " to " << "camera_optical";
      ROS_WARN("%s", ss.str().c_str());
    }
  } while(ros::ok() && !transformed);
  ROS_INFO("Found tranform");




  ros::Rate rate(60);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};
