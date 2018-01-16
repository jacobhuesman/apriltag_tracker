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
#include <camera_info_manager/camera_info_manager.h>

#include <apriltag_tracker/AprilTagDetection.h>
#include <apriltag_tracker/AprilTagDetectionArray.h>
#include <apriltag_tracker/ATTDiagnostics.h>

#include <apriltag_tracker.h>
#include <raspicam/raspicamtypes.h>


raspicam::RaspiCam *camera;
boost::mutex *camera_mutex;

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
const bool publish_pose_estimate = true;

void trackerThread(ros::NodeHandle nh, sensor_msgs::CameraInfo camera_info, uint8_t thread_id)
{
  timing_mutex.lock();
  AprilTagTracker::AprilTagTracker tracker(camera_info, camera, camera_mutex);


  std_msgs::Header image_header;
  image_header.frame_id = "camera";
  cv_bridge::CvImage cv_bridge_image(image_header, sensor_msgs::image_encodings::MONO8, *tracker.image_gs);

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
    tracker.getImage();
    timing_mutex.lock();
    global_time_stamp[1] = global_time_stamp[0];
    global_time_stamp[0] = std::chrono::system_clock::now();
    timing_mutex.unlock();
    time_stamp[1] = std::chrono::system_clock::now();
    tracker.processImage();
    time_stamp[2] = std::chrono::system_clock::now();
    //tracker.adjustServo();

    // Send Transforms
    time_stamp[3] = std::chrono::system_clock::now();
    apriltag_tracker::AprilTagDetectionArray tag_detection_array;
    tracker.calculateAprilTagTransforms(&tag_detection_array); // TODO publish this array
    time_stamp[4] = std::chrono::system_clock::now();
    for (int i = 0; i < tag_detection_array.detections.size(); i++)
    {
      tf2::Stamped<tf2::Transform> tag_transform;
      geometry_msgs::TransformStamped tag_message;
      tf2::fromMsg(tag_detection_array.detections[i].pose, tag_transform);
      tag_message = tf2::toMsg(tag_transform);

      tag_message.header.frame_id = "camera_optical";
      tag_message.child_frame_id = std::string("tag") + std::to_string(tag_detection_array.detections[i].id) + "_estimate";

      tag_message.header.stamp = ros::Time::now();
      pubs->tf.sendTransform(tag_message);
    }
    time_stamp[5] = std::chrono::system_clock::now();
    pubs->detections.publish(tag_detection_array);

    // Publish images
    time_stamp[6] = std::chrono::system_clock::now();
    if (publish_plain_image)
    {
      pubs->image.publish(cv_bridge_image.toImageMsg());
    }
    time_stamp[7] = std::chrono::system_clock::now();
    tracker.drawDetections(tracker.image_gs);
    time_stamp[8] = std::chrono::system_clock::now();
    pubs->detections_image.publish(cv_bridge_image.toImageMsg());

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
    diagnostics.local_timing.image_capture_time       = cv_bridge_image.header.stamp;
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
  camera_info_manager::CameraInfoManager camera_info(nh, "camera",
                                                     "package://apriltag_tracker/calibrations/${NAME}.yaml");
  image_transport::ImageTransport it(nh);
  pubs = new Publishers;
  pubs->detections = nh.advertise<apriltag_tracker::AprilTagDetectionArray>("tag_detections", 1);
  pubs->diagnostics = nh.advertise<apriltag_tracker::ATTDiagnostics>("diagnostics", 1);
  pubs->image = it.advertise("camera_image", 1);
  pubs->detections_image = it.advertise("tag_detections_image", 1);


  if (publish_pose_estimate)
  {

  }

  int ss;
  int brt;
  int crt;
  int shp;
  int sat;
  int bv;

  /*nh.getParam("/apriltag_detector/ss", ss);
  nh.getParam("/apriltag_detector/brt", brt);
  nh.getParam("/apriltag_detector/crt", crt);*/

  std::cout << "ss: " << ss << ", brt: " << brt << std::endl;


  // Initialize tracker
  if (!camera_info.isCalibrated())
  {
    ROS_WARN("Using uncalibrated camera!");
  }

  // Initialize camera
  // TODO parametrize and load from camera calibration
  camera = new raspicam::RaspiCam;
  camera->setWidth(640);
  camera->setHeight(480);
  camera->setShutterSpeed(8000); // 10000, 8000
  camera->setBrightness(80);  // 70, 80
  camera->setContrast(100);    // 100
  camera->setVideoStabilization(true);
  camera->setFormat(raspicam::RASPICAM_FORMAT_GRAY);
  camera->setRotation(180);
  camera->open();
  while(!camera->isOpened()) // TODO add failure condition
  {
    std::cout << "Unable to open camera, waiting 0.5 seconds and trying again..." << std::endl;
    usleep(500000);
    camera->open();
  }

  // Start threads
  camera_mutex = new boost::mutex;
  boost::thread thread0(trackerThread, nh, camera_info.getCameraInfo(), 0);
  boost::thread thread1(trackerThread, nh, camera_info.getCameraInfo(), 1);
  boost::thread thread2(trackerThread, nh, camera_info.getCameraInfo(), 2);
  boost::thread thread3(trackerThread, nh, camera_info.getCameraInfo(), 3);
  thread0.join();
  thread1.join();
  thread2.join();
  thread3.join();
}

