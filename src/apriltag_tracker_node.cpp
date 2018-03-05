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

using namespace AprilTagTracker;

struct Publishers
{
  ros::Publisher pose;
  ros::Publisher diagnostics;
  ros::Publisher transforms;
  image_transport::Publisher image;
};
Publishers *pubs;

const bool servo_track_tag = true;
const bool publish_pose_estimate = false;

void trackerThread(HostCommLayer::Dynamixel *servo, AprilTagTracker::TransformsCache transforms_cache,
                   std::vector<AprilTagTracker::Tag> *tag_info, apriltag_tracker::Camera *camera)
{
  AprilTagTracker::Timers timer;
  AprilTagTracker::AprilTagTracker tracker(camera->getCameraInfo(), tag_info, transforms_cache);

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

void servoThread(HostCommLayer::Dynamixel *servo, std::vector<AprilTagTracker::Tag> *tag_info)
{
  ros::Rate rate(30);
  while(ros::ok())
  {
    rate.sleep();
    /*servo->updatePosition();
    if (servo_track_tag)
    {
      double theta = 0.0;
      ros::Time last_tag_update;
      try
      {
        for (int i = 0; i < tag_info->size(); i++)
        {
          if ((*tag_info)[i].getID() == 1)
          {
            theta = (*tag_info)[i].getAngleFromCenter(5);
          }
        }
        servo->adjustCamera(servo->calculateDesiredVelocity(theta));
        pubs->transforms.publish(servo->getTransformMsg());
      }
      catch (unable_to_find_transform_error &e)
      {
        ROS_WARN(e.what());
        servo->scan();
        continue;
      }
    }*/
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

  // Initialize tags and transforms_cache cache
  std::vector<AprilTagTracker::Tag> *tag_info = new std::vector<AprilTagTracker::Tag>;
  Tag tag1(1, 2, 0.42545);
  Tag tag3(3, 1, 0.42545);
  tag_info->push_back(tag1);
  tag_info->push_back(tag3);
  TransformsCache::initializeTagVector(tag_info);
  AprilTagTracker::TransformsCache transforms_cache(nh);

  // Initialize servo
  HostCommLayer::Dynamixel *servo = new HostCommLayer::Dynamixel(0x11);

  // Initialize Camera Objects
  apriltag_tracker::CameraMaster camera_master(nh);
  apriltag_tracker::Camera *camera0 = camera_master.generateCameraObject();
  apriltag_tracker::Camera *camera1 = camera_master.generateCameraObject();
  apriltag_tracker::Camera *camera2 = camera_master.generateCameraObject();
  apriltag_tracker::Camera *camera3 = camera_master.generateCameraObject();

  // Start threads
  boost::thread thread0(trackerThread, servo, transforms_cache, tag_info, camera0); usleep(1000);
  boost::thread thread1(trackerThread, servo, transforms_cache, tag_info, camera1); usleep(1000);
  boost::thread thread2(trackerThread, servo, transforms_cache, tag_info, camera2); usleep(1000);
  boost::thread thread3(trackerThread, servo, transforms_cache, tag_info, camera3);
  boost::thread thread4(servoThread, servo, tag_info);
  ros::spin();

  // Process images until quit
  thread0.join();
  thread1.join();
  thread2.join();
  thread3.join();
  thread4.join();
}