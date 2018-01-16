#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
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
  static tf2_ros::TransformBroadcaster br;

  dynamic_reconfigure::Server<apriltag_tracker::TFTestConfig> dr_server;
  dynamic_reconfigure::Server<apriltag_tracker::TFTestConfig>::CallbackType f;
  f = boost::bind(&dynamicReconfigureCallback, _1, _2);
  dr_server.setCallback(f);

  tf2::Quaternion q;

  q.setRPY(-M_PI_2, 0, -M_PI_2);
  geometry_msgs::TransformStamped servo_joint_to_optical_link_tf;
  servo_joint_to_optical_link_tf.header.seq = 0;
  servo_joint_to_optical_link_tf.header.frame_id = "servo_joint";
  servo_joint_to_optical_link_tf.child_frame_id = "camera_optical";
  servo_joint_to_optical_link_tf.transform.translation.x = 9.0e-3;
  servo_joint_to_optical_link_tf.transform.translation.y = 0.0;
  servo_joint_to_optical_link_tf.transform.translation.z = 25.0e-3;
  servo_joint_to_optical_link_tf.transform.rotation.x = q.getX();
  servo_joint_to_optical_link_tf.transform.rotation.y = q.getY();
  servo_joint_to_optical_link_tf.transform.rotation.z = q.getZ();
  servo_joint_to_optical_link_tf.transform.rotation.w = q.getW();

  q.setRPY(0.0, 0.0, 0.0);
  geometry_msgs::TransformStamped servo_base_link_to_servo_joint_tf;
  servo_base_link_to_servo_joint_tf.header.seq = 0;
  servo_base_link_to_servo_joint_tf.header.frame_id = "servo_base_link";
  servo_base_link_to_servo_joint_tf.child_frame_id = "servo_joint";
  servo_base_link_to_servo_joint_tf.transform.translation.x = 24.15e-3;
  servo_base_link_to_servo_joint_tf.transform.translation.y = 0.0;
  servo_base_link_to_servo_joint_tf.transform.translation.z = 32.5e-3;
  servo_base_link_to_servo_joint_tf.transform.rotation.x = q.getX();
  servo_base_link_to_servo_joint_tf.transform.rotation.y = q.getY();
  servo_base_link_to_servo_joint_tf.transform.rotation.z = q.getZ();
  servo_base_link_to_servo_joint_tf.transform.rotation.w = q.getW();

  ros::Rate rate(60);
  while(ros::ok())
  {
    q.setRPY(0.0, 0.0, yaw);
    servo_base_link_to_servo_joint_tf.transform.rotation.x = q.getX();
    servo_base_link_to_servo_joint_tf.transform.rotation.y = q.getY();
    servo_base_link_to_servo_joint_tf.transform.rotation.z = q.getZ();
    servo_base_link_to_servo_joint_tf.transform.rotation.w = q.getW();

    servo_joint_to_optical_link_tf.header.stamp = ros::Time::now();
    servo_base_link_to_servo_joint_tf.header.stamp = ros::Time::now();

    servo_joint_to_optical_link_tf.header.seq++;
    servo_base_link_to_servo_joint_tf.header.seq++;

    br.sendTransform(servo_joint_to_optical_link_tf);
    br.sendTransform(servo_base_link_to_servo_joint_tf);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
};
