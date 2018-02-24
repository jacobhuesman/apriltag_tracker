#include <iostream>
#include <sstream>
#include <cmath>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <apriltag_tracker/AprilTagDetection.h>
#include <apriltag_tracker/AprilTagDetectionArray.h>
#include <apriltag_tracker/ATTDiagnostics.h>

#include <apriltag_tracker.h>
#include <timers.h>
#include <dynamixel_host_layer.h>
#include <camera.h>
#include <errors.h>

struct Publishers
{
  ros::Publisher pose;
  ros::Publisher diagnostics;
  ros::Publisher transforms;
  image_transport::Publisher image;
};
Publishers *pubs;

const bool track_servo = true;
const bool publish_pose_estimate = true;

std::vector<AprilTagTracker::Tag> *tag_info;

// TODO use some kind of xml and load from there
void initializeTagInfoVector(std::vector<AprilTagTracker::Tag> *tag_info)
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  AprilTagTracker::Tag tag1(1, 2, 0.42545);
  AprilTagTracker::Tag tag3(3, 1, 0.42545);
  AprilTagTracker::Tag tag4(4, 1, 0.203);

  tag_info->push_back(tag1);
  tag_info->push_back(tag3);
  tag_info->push_back(tag4);

  for (int i = 0; i < tag_info->size(); i++)
  {
    std::string tag_name = "tag" + std::to_string((*tag_info)[i].getID());
    bool transformed;
    do
    {
      transformed = tfBuffer.canTransform("map", tag_name, ros::Time(0), ros::Duration(1));
      if (!transformed)
      {
        std::stringstream ss;
        ss << "Unable to find transform from " << "map" << " to " << tag_name;
        ROS_WARN("%s", ss.str().c_str());
      }
    } while(ros::ok() && !transformed);
    ROS_INFO("Found transform for tag%i", (*tag_info)[i].getID());

    geometry_msgs::TransformStamped transform_msg;
    transform_msg = tfBuffer.lookupTransform("map", tag_name, ros::Time(0));
    tf2::Transform map_to_tag_tf;
    tf2::fromMsg(transform_msg.transform, map_to_tag_tf);
    (*tag_info)[i].setMapToTagTf(map_to_tag_tf);
  }
}

void trackerThread(ros::NodeHandle nh, HostCommLayer::Dynamixel *servo, AprilTagTracker::TransformsCache transforms,
                   apriltag_tracker::Camera *camera, uint8_t thread_id)
{
  AprilTagTracker::Timers timer;
  AprilTagTracker::AprilTagTracker tracker(camera->getCameraInfo(), tag_info, transforms);

  // Note: Entire loop must take less than 33ms
  while(ros::ok())
  {
    // Get image, process it, and track
    timer.get_image.start();
    camera->grabImage();
    timer.get_image.stop();

    timer.process_image.start();
    tracker.processImage(camera->getImagePtr(), camera->getSeq(), camera->getCaptureTime(),
                         servo->getStampedTransform());
    timer.process_image.stop();


    timer.adjust_servo.start();
    try
    {
      servo->updateDesiredVelocity(tracker.getDesiredServoVelocity());
    }
    catch(AprilTagTracker::unable_to_find_transform_error &e)
    {
      ROS_WARN("%s", e.what());
    }
    timer.adjust_servo.stop();

    // Send Transforms
    timer.publish_tag_transforms.start();
    std::vector<geometry_msgs::TransformStamped> tag_transforms = tracker.getTagTransforms();
    for (int i = 0; i < tag_transforms.size(); i++)
    {
      pubs->transforms.publish(tag_transforms[i]);
    }
    timer.publish_tag_transforms.stop();

    timer.publish_transforms.start();
    if (publish_pose_estimate)
    {
      try
      {
        geometry_msgs::PoseStamped pose_estimate_msg;
        tracker.estimateRobotPose(&pose_estimate_msg);
        pubs->pose.publish(pose_estimate_msg);
      }
      catch (AprilTagTracker::unable_to_find_transform_error &e)
      {
        ROS_WARN("%s", e.what());
      }
    }
    timer.publish_transforms.stop();

    timer.draw_detections.start();
    tracker.drawDetections(camera->getImagePtr());
    timer.draw_detections.stop();

    timer.publish_detections_image.start();
    pubs->image.publish(camera->getImageMsg());
    timer.publish_detections_image.stop();

    // Handle callbacks
    timer.spin.start();
    ros::spinOnce();
    timer.spin.stop();

    // Publish timing info
    apriltag_tracker::ATTLocalTiming timing_msg = timer.getTimingMsg();
    timing_msg.image_capture_time = ros::Time::now();
    pubs->diagnostics.publish(timing_msg);
  }
}

void servoThread(HostCommLayer::Dynamixel *servo)
{
  ros::Rate rate(30);
  while(ros::ok())
  {
    if (track_servo)
    {
      // Start scanning if enough time has elapsed since last capture
      uint8_t status;
      ros::Duration difference = ros::Time::now() - servo->getLastVelocityUpdate();
      if ((difference.sec > 0) ||
          (difference.nsec > 5e8)) // More than half a second has elapsed since an apriltag was captured
      {
        status = servo->scan();
      }
      else
      {
        status = servo->adjustCamera(servo->getDesiredVelocity());
      }
      if (status == CL_OK)
      {
        pubs->transforms.publish(servo->getTransformMsg());
      }
    }
    rate.sleep();
  }
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "position_sensor");
  ros::NodeHandle nh("~");

  image_transport::ImageTransport it(nh);
  pubs = new Publishers;
  pubs->diagnostics = nh.advertise<apriltag_tracker::ATTLocalTiming>("info/timing_diagnostics", 30);
  pubs->pose = nh.advertise<geometry_msgs::PoseStamped>("pose_estimate", 30);
  pubs->transforms = nh.advertise<geometry_msgs::TransformStamped>("transforms", 30);
  pubs->image = it.advertise("image", 1);

  // Initialize tags
  tag_info = new std::vector<AprilTagTracker::Tag>;
  initializeTagInfoVector(tag_info);

  // Get camera_transform
  // TODO parametrize
  // TODO separate into class?
  AprilTagTracker::TransformsCache transforms;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  std::stringstream ss;
  bool transformed = false;
  do
  {
    transformed = tfBuffer.canTransform("camera_optical", "servo_joint", ros::Time(0), ros::Duration(1));
    if (!transformed)
    {
      ss.clear();
      ss << "Unable to find transform from " << "camera_optical" << " to " << "base_link";
      ROS_WARN("%s", ss.str().c_str());
    }
  } while(ros::ok() && !transformed);
  ss.clear();
  ss << "Found transform from " << "camera_optical" << " to " << "servo_joint";
  ROS_INFO("%s", ss.str().c_str());
  geometry_msgs::TransformStamped transform_msg;
  transform_msg = tfBuffer.lookupTransform("camera_optical", "servo_joint", ros::Time(0));
  tf2::fromMsg(transform_msg.transform, transforms.camera_optical_to_servo_joint);
  do
  {
    transformed = tfBuffer.canTransform("servo_base_link", "base_link", ros::Time(0), ros::Duration(1));
    if (!transformed)
    {
      ss.clear();
      ss << "Unable to find transform from " << "servo_base_link" << " to " << "base_link";
      ROS_WARN("%s", ss.str().c_str());
    }
  } while(ros::ok() && !transformed);
  ss.clear();
  ss << "Found transform from " << "servo_base_link" << " to " << "base_link";
  ROS_INFO("%s", ss.str().c_str());
  transform_msg = tfBuffer.lookupTransform("servo_base_link", "base_link", ros::Time(0));
  tf2::fromMsg(transform_msg.transform, transforms.servo_base_link_to_base_link);

  // Initialize servo
  HostCommLayer::Dynamixel *servo = new HostCommLayer::Dynamixel(0x11);

  // Initialize Camera Objects
  apriltag_tracker::CameraMaster camera_master(nh);
  apriltag_tracker::Camera *camera0 = camera_master.generateCameraObject();
  apriltag_tracker::Camera *camera1 = camera_master.generateCameraObject();
  apriltag_tracker::Camera *camera2 = camera_master.generateCameraObject();
  apriltag_tracker::Camera *camera3 = camera_master.generateCameraObject();


  // Start threads
  boost::thread thread0(trackerThread, nh, servo, transforms, camera0, 0); usleep(1000);
  boost::thread thread1(trackerThread, nh, servo, transforms, camera1, 1); usleep(1000);
  boost::thread thread2(trackerThread, nh, servo, transforms, camera2, 2); usleep(1000);
  boost::thread thread3(trackerThread, nh, servo, transforms, camera3, 3);
  boost::thread thread4(servoThread, servo);

  ros::spin();

  thread0.join();
  thread1.join();
  thread2.join();
  thread3.join();
  thread4.join();
}

