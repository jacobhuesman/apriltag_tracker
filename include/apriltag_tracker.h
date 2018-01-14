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

namespace AprilTagTracker
{

typedef struct {
  float fx;
  float fy;
  float px;
  float py;
  float width;
  float height;
  cv::Point2f fov;
  cv::Point2f optical_center;
  cv::Point2f degrees_per_pixel;

} CameraProperties;

class AprilTagTracker
{
public:
  AprilTagTracker();
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
