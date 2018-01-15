#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

#include <apriltag_tracker/TFTestConfig.h>

double yaw = 0.0;

void dynamicReconfigureCallback(apriltag_tracker::TFTestConfig &config, uint32_t level)
{
  yaw = config.yaw;
  ROS_INFO("Yaw set to: %f", yaw);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tftest");
  ros::NodeHandle nh;
  static tf::TransformBroadcaster br;

  dynamic_reconfigure::Server<apriltag_tracker::TFTestConfig> dr_server;
  dynamic_reconfigure::Server<apriltag_tracker::TFTestConfig>::CallbackType f;
  f = boost::bind(&dynamicReconfigureCallback, _1, _2);
  dr_server.setCallback(f);

  tf::Quaternion q;

  tf::Transform tf_servo_joint_to_optical_link;
  tf_servo_joint_to_optical_link.setOrigin(tf::Vector3(9.0e-3, 0.0, 25.0e-3));
  q.setRPY(-M_PI_2, 0, -M_PI_2);
  tf_servo_joint_to_optical_link.setRotation(q);

  tf::Transform tf_servo_base_link_to_servo_joint;
  tf_servo_base_link_to_servo_joint.setOrigin(tf::Vector3(24.15e-3, 0, 32.5e-3));



  ros::Rate rate(60);
  while(ros::ok())
  {
    q.setRPY(0.0, 0.0, yaw);
    tf_servo_base_link_to_servo_joint.setRotation(q);

    br.sendTransform(tf::StampedTransform(tf_servo_joint_to_optical_link, ros::Time::now(), "servo_joint", "camera_optical"));
    br.sendTransform(tf::StampedTransform(tf_servo_base_link_to_servo_joint, ros::Time::now(), "servo_base_link", "servo_joint"));
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};
