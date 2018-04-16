#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static tf2_ros::TransformBroadcaster *br;

void transformsCallback(const geometry_msgs::TransformStamped::ConstPtr& transform)
{
  geometry_msgs::TransformStamped synced_transform = *transform;
  synced_transform.header.stamp = ros::Time::now(); // Don't use this technique for anything other than visuals
  br->sendTransform(synced_transform);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "transforms_publisher");
  ros::NodeHandle nh;
  br = new tf2_ros::TransformBroadcaster();
  ros::Subscriber sub0 = nh.subscribe("/node0/transforms", 1000, transformsCallback);
  ros::Subscriber sub1 = nh.subscribe("/node1/transforms", 1000, transformsCallback);
  ros::spin();

  return 0;
};
