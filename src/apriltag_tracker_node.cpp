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
#include <raspicam/raspicamtypes.h>


struct Publishers
{
  tf2_ros::TransformBroadcaster tf;
  ros::Publisher detections;
  ros::Publisher diagnostics;
  image_transport::Publisher image;
  image_transport::Publisher detections_image;
};

Publishers *pubs;

boost::mutex timing_mutex;
long running_count = 0;
long iterations = 0;
long max_processing_time = 0;
long max_image_grab_dt = 0;
long misses = 0;
std::chrono::system_clock::time_point global_time_stamp[2];

const bool publish_plain_image = false;
const bool track_servo = true;
const bool publish_pose_estimate = true;

std::vector<AprilTagTracker::TagInfo> *tag_info;

// TODO use some kind of xml and load from there
void initializeTagInfoVector(std::vector<AprilTagTracker::TagInfo> *tag_info)
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  AprilTagTracker::TagInfo tag;

  // Tag 1
  tag.id = 1;
  tag.seq = 0;
  tag.priority = 0;
  tag.size = 0.42545;
  tag.mutex = new boost::mutex;
  tag_info->push_back(tag);

  // Tag 4
  tag.id = 4;
  tag.seq = 0;
  tag.priority = 0;
  tag.size = 0.203;
  tag.mutex = new boost::mutex;
  tag_info->push_back(tag);

  // Tag 3
  tag.id = 3;
  tag.seq = 0;
  tag.priority = 0;
  tag.size = 0.203;
  tag.mutex = new boost::mutex;
  tag_info->push_back(tag);

  for (int i = 0; i < tag_info->size(); i++)
  {
    std::string tag_name = "tag" + std::to_string((*tag_info)[i].id);
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
    ROS_INFO("Found transform for tag%i", (*tag_info)[i].id);

    geometry_msgs::TransformStamped transform_msg;
    transform_msg = tfBuffer.lookupTransform("map", tag_name, ros::Time(0));
    tf2::fromMsg(transform_msg.transform, (*tag_info)[i].map_to_tag_tf);
  }
}


void trackerThread(ros::NodeHandle nh, HostCommLayer::Dynamixel *servo, AprilTagTracker::TransformsCache transforms,
                   apriltag_tracker::Camera *camera, uint8_t thread_id)
{
  timing_mutex.lock();
  AprilTagTracker::AprilTagTracker tracker(camera, servo, tag_info, transforms);

  // Timing info
  std::chrono::system_clock::time_point time_stamp[12];
  std::chrono::microseconds calc_time[12];

  apriltag_tracker::ATTDiagnostics diagnostics;
  diagnostics.thread = thread_id;
  timing_mutex.unlock();

  // Note: Entire loop must take less than 33ms
  while(ros::ok())
  {
    // Get image, process it, and track
    time_stamp[0] = std::chrono::system_clock::now();
    tracker.camera->grabImage();
    timing_mutex.lock();
    global_time_stamp[1] = global_time_stamp[0];
    global_time_stamp[0] = std::chrono::system_clock::now();
    timing_mutex.unlock();
    time_stamp[1] = std::chrono::system_clock::now();
    tracker.processImage();
    time_stamp[2] = std::chrono::system_clock::now();
    if (track_servo)
    {
      tracker.adjustServo();
    }

    // Send Transforms
    time_stamp[3] = std::chrono::system_clock::now();
    apriltag_tracker::AprilTagDetectionArray tag_detection_array;
    tracker.calculateTransforms(&tag_detection_array);
    time_stamp[4] = std::chrono::system_clock::now();
    for (int i = 0; i < tag_detection_array.detections.size(); i++)
    {
      pubs->tf.sendTransform(tag_detection_array.detections[i].transform);
    }
    geometry_msgs::TransformStamped pose_estimate_msg;
    tracker.estimateRobotPose(&pose_estimate_msg);

    if (publish_pose_estimate)
    {
      pubs->tf.sendTransform(tracker.servo->getTransformMsg());
      pubs->tf.sendTransform(pose_estimate_msg); // TODO should be pose msg
    }
    time_stamp[5] = std::chrono::system_clock::now();
    pubs->detections.publish(tag_detection_array);

    // Publish images
    time_stamp[6] = std::chrono::system_clock::now();
    if (publish_plain_image)
    {
      pubs->image.publish(tracker.camera->getImageMsg());
    }
    time_stamp[7] = std::chrono::system_clock::now();
    tracker.drawDetections();
    time_stamp[8] = std::chrono::system_clock::now();
    pubs->detections_image.publish(tracker.camera->getImageMsg());

    time_stamp[9] = std::chrono::system_clock::now();
    ros::spinOnce();
    time_stamp[10] = std::chrono::system_clock::now();
    // TODO look into using image_geometry ROS library

    // Timing info
    for (int i = 0; i < 10; i++)
    {
      calc_time[i] = std::chrono::duration_cast<std::chrono::microseconds>(time_stamp[i+1] - time_stamp[i]);
    }
    calc_time[10] = std::chrono::duration_cast<std::chrono::microseconds>(time_stamp[10] - time_stamp[0]);
    calc_time[11] = std::chrono::duration_cast<std::chrono::microseconds>(time_stamp[10] - time_stamp[1]);
    running_count += calc_time[11].count();
    iterations++;
    diagnostics.local_timing.image_capture_time       = camera->getCaptureTime();
    diagnostics.local_timing.get_image                = calc_time[0].count();
    diagnostics.local_timing.process_image            = calc_time[1].count();
    diagnostics.local_timing.adjust_servo             = calc_time[2].count();
    diagnostics.local_timing.calculate_transforms     = calc_time[3].count();
    diagnostics.local_timing.publish_transforms       = calc_time[4].count();
    diagnostics.local_timing.publish_detections_array = calc_time[5].count();
    diagnostics.local_timing.publish_plain_image      = calc_time[6].count();
    diagnostics.local_timing.draw_detections          = calc_time[7].count();
    diagnostics.local_timing.publish_detections_image = calc_time[8].count();
    diagnostics.local_timing.spin                     = calc_time[9].count();
    diagnostics.local_timing.total_time               = calc_time[10].count();
    diagnostics.local_timing.processing_time          = calc_time[11].count();

    // Global values
    timing_mutex.lock();
    diagnostics.global_timing.iteration = iterations;
    diagnostics.global_timing.average_processing_time = running_count / iterations;
    max_processing_time = std::max(max_processing_time, (long)(calc_time[11].count()));
    diagnostics.global_timing.max_processing_time = max_processing_time;
    diagnostics.global_timing.image_grab_dt =
        std::chrono::duration_cast<std::chrono::microseconds>(global_time_stamp[0] - global_time_stamp[1]).count();
    max_image_grab_dt = std::max(max_image_grab_dt, (long)diagnostics.global_timing.image_grab_dt);
    diagnostics.global_timing.max_image_grab_dt = max_image_grab_dt;
    if (diagnostics.global_timing.image_grab_dt > 40000)
    {
      diagnostics.global_timing.misses = ++misses;
    }
    diagnostics.global_timing.percent_missed =
        ((double)diagnostics.global_timing.misses) / ((double)diagnostics.global_timing.iteration) * 100.0;
    pubs->diagnostics.publish(diagnostics);
    timing_mutex.unlock();

    //std::string image_path = "/home/nrmc/ws/test_images/image" + std::to_string(iterations) + ".jpg";
    //cv::imwrite(image_path, captured_image_mat);

  }
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "apriltag_detector");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  pubs = new Publishers;
  pubs->detections = nh.advertise<apriltag_tracker::AprilTagDetectionArray>("tag_detections", 1);
  pubs->diagnostics = nh.advertise<apriltag_tracker::ATTDiagnostics>("diagnostics", 1);
  pubs->image = it.advertise("camera_image", 1);
  pubs->detections_image = it.advertise("tag_detections_image", 1);

  // Initialize tags
  tag_info = new std::vector<AprilTagTracker::TagInfo>;
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
  boost::thread thread0(trackerThread, nh, servo, transforms, camera0, 0);
  boost::thread thread1(trackerThread, nh, servo, transforms, camera1, 1);
  boost::thread thread2(trackerThread, nh, servo, transforms, camera2, 2);
  boost::thread thread3(trackerThread, nh, servo, transforms, camera3, 3);

  ros::spin();

  thread0.join();
  thread1.join();
  thread2.join();
  thread3.join();
}

