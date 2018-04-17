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

#include <apriltag_tracker/apriltag_tracker.h>
#include <apriltag_tracker/timers.h>
#include <dynamixel.h>
#include <apriltag_tracker/camera.h>

using namespace apriltag_tracker;

struct Publishers
{
  ros::Publisher pose;
  ros::Publisher diagnostics;
  ros::Publisher transforms;
  image_transport::Publisher image;
};
Publishers *pubs;

void trackerThread(apriltag_tracker::Dynamixel *servo, TransformsCache transforms_cache,
                   std::vector<Tag> *tag_info, AprilTagTrackerConfig *tracker_config,
                   Camera *camera)
{
  Timers timer;
  AprilTagTracker tracker(camera->getCameraInfo(), tag_info, tracker_config, transforms_cache);

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
    try
    {
      geometry_msgs::PoseStamped pose_estimate_msg;
      tracker.estimateRobotPose(&pose_estimate_msg);
      pubs->pose.publish(pose_estimate_msg);
    }
    catch (apriltag_tracker::unable_to_find_transform_error &e)
    {
      ROS_WARN("%s", e.what());
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

void servoThread(apriltag_tracker::Dynamixel *servo, std::vector<apriltag_tracker::Tag> *tag_info)
{
  ros::Rate rate(30);
  while(ros::ok())
  {
    rate.sleep();
    try
    {
      servo->updatePosition();
    }
    catch (cl_error &e)
    {
      ROS_WARN("%s", e.what());
    }
    double theta = 0.0;
    ros::Time last_tag_update;
    try
    {
      for (int i = 0; i < tag_info->size(); i++)
      {
        if ((*tag_info)[i].getID() == 1)
        {
          theta = (*tag_info)[i].getAngleFromCenter(1);
        }
      }
      servo->adjustCamera(-servo->calculateDesiredVelocity(theta));
    }
    catch (unable_to_find_transform_error &e)
    {
      ROS_WARN("%s | Starting scan", e.what());
      try
      {
        servo->scan();
      }
      catch (cl_error &e)
      {
        ROS_WARN("%s", e.what());
      }
    }
    catch (cl_error &e)
    {
      ROS_WARN("%s", e.what());
    }
    pubs->transforms.publish(servo->getTransformMsg());
  }
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "node0");
  ros::NodeHandle nh("~");
  ros::NodeHandle imt_nh("~image_transport");
  image_transport::ImageTransport it(imt_nh);
  pubs = new Publishers;
  pubs->diagnostics = nh.advertise<ATTLocalTiming>("info/timing_diagnostics", 30);
  pubs->pose = nh.advertise<geometry_msgs::PoseStamped>("pose_estimate", 30);
  pubs->transforms = nh.advertise<geometry_msgs::TransformStamped>("transforms", 30);
  pubs->image = it.advertise("image", 1);

  // Initialize tags and transforms_cache cache
  std::vector<Tag> *tag_info = new std::vector<Tag>;
  Tag tag0(0, 1, 0.4318);
  Tag tag1(1, 2, 0.4318);
  tag_info->push_back(tag0);
  tag_info->push_back(tag1);
  TransformsCache::initializeTagVector(tag_info);
  TransformsCache transforms_cache(nh);

  // Initialize servo
  ros::NodeHandle dyn_nh("~servo");
  std::string name = ros::this_node::getName();
  Dynamixel *servo = new Dynamixel(0x11, name + "_dynamixel", name + "_camera_mount");
  dynamic_reconfigure::Server<apriltag_tracker::DynamicServoConfig> server(dyn_nh);
  dynamic_reconfigure::Server<DynamicServoConfig>::CallbackType f;
  f = boost::bind(&Dynamixel::reconfigureCallback, servo, _1, _2 );
  server.setCallback(f);

  // Initialize Camera Objects
  if (mraa_get_platform_type() != MRAA_RASPBERRY_PI)
  {
    throw cl_host_error("Host must be a Raspberry PI");
  }
  ros::NodeHandle cam_nh("~camera");
  CameraMaster camera_master(cam_nh);
  Camera *camera0 = camera_master.generateCameraObject();
  Camera *camera1 = camera_master.generateCameraObject();
  Camera *camera2 = camera_master.generateCameraObject();
  Camera *camera3 = camera_master.generateCameraObject();

  // Initialize apriltag tracker config object
  AprilTagTrackerConfig *tracker_config = new AprilTagTrackerConfig();

  // Start threads
  boost::thread thread0(trackerThread, servo, transforms_cache, tag_info, tracker_config, camera0); usleep(1000);
  boost::thread thread1(trackerThread, servo, transforms_cache, tag_info, tracker_config, camera1); usleep(1000);
  boost::thread thread2(trackerThread, servo, transforms_cache, tag_info, tracker_config, camera2); usleep(1000);
  boost::thread thread3(trackerThread, servo, transforms_cache, tag_info, tracker_config, camera3);
  boost::thread thread4(servoThread, servo, tag_info);
  ros::spin();

  // Process images until quit
  thread0.join();
  thread1.join();
  thread2.join();
  thread3.join();
  thread4.join();
}