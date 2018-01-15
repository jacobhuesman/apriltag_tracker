#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv){
  ros::init(argc, argv, "tftest");
  ros::NodeHandle nh;

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.5));
  tf::Quaternion q;
  q.setRPY(-M_PI_2, 0, -M_PI_2);
  transform.setRotation(q);

  ros::Rate rate(60);
  while(ros::ok())
  {
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "camera_optical"));
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};
