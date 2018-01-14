#ifndef PROJECT_APRILTAG_TRACKER_H
#define PROJECT_APRILTAG_TRACKER_H

#include <chrono>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <apriltag_tracker/AprilTagDetection.h>
#include <apriltag_tracker/AprilTagDetectionArray.h>

#include <TagDetector.h>
#include <raspicam/raspicam.h>

#include <host_comm_layer.h>
#include <sensor_msgs/CameraInfo.h>

namespace AprilTagTracker
{

// See sensor_msgs::CameraInfo for descriptions
struct CameraProperties
{
  // Derived from CameraInfo message
  uint32_t height;
  uint32_t width;
  std::vector<float> D; // Distortion Parameters
  cv::Matx33f K;        // Intrinsic camera matrix for the raw (distorted) images
  cv::Matx33f P;        // Projection/camera matrix, intrinsic (camera) matrix for processed (rectified) image.
  cv::Matx34f R;        // Rectification matrix (stereo cameras only)

  // TODO check to make sure P has the right row/column order

  uint32_t binning_x;
  uint32_t binning_y;
  // TODO add ROI

  // Additional parameters
  cv::Point2f fov;
  cv::Point2f optical_center;
  cv::Point2f degrees_per_pixel;
};

class AprilTagTracker
{
public:
  AprilTagTracker();
  AprilTagTracker(sensor_msgs::CameraInfo camera_info);
  ~AprilTagTracker();

  Eigen::Matrix4f getRelativeTransform(const cv::Point2f tagPts[]);
  void drawDetections(cv::Mat *image);
  void getAndProcessImage();
  void getAndProcessImage(cv::Mat *image);
  void adjustServo();
  void outputTimingInfo();
  void populateAprilTagDetectionsArray(apriltag_tracker::AprilTagDetectionArray *tag_detection_array);

  cv::Mat *image_gs;
  HostCommLayer::Dynamixel *servo;
  CameraProperties camera_properties;

private:
  // Tag
  TagDetectorParams tag_params;
  TagFamily *tag_family;
  TagDetector *tag_detector;
  TagDetectionArray tag_detections;
  float tag_size; //TODO Create an array of tags, their sizes and their priorities

  // Camera
  raspicam::RaspiCam camera;

  // Transform
  Eigen::Matrix4f transform_matrix;
  Eigen::Vector3f rotation_matrix;
  Eigen::Quaternion<float> rotation;
  geometry_msgs::Pose pose;

  // Timing
  std::chrono::system_clock::time_point get_and_process_data_time_points[4];
  std::chrono::microseconds calc_time[10];
  long running_count = 0;
  long iterations = 0;
  ros::Time capture_time;

};

}

#endif //PROJECT_APRILTAG_TRACKER_H
