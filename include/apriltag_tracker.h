#ifndef PROJECT_APRILTAG_TRACKER_H
#define PROJECT_APRILTAG_TRACKER_H

#include <chrono>
#include <boost/thread/thread.hpp>
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
  std::vector<double> D; // Distortion Parameters
  cv::Matx33d K;        // Intrinsic camera matrix for the raw (distorted) images
  cv::Matx33d P;        // Projection/camera matrix, intrinsic (camera) matrix for processed (rectified) image.
  cv::Matx34d R;        // Rectification matrix (stereo cameras only)

  // TODO check to make sure P has the right row/column order

  uint32_t binning_x;
  uint32_t binning_y;
  // TODO add ROI

  // Additional parameters
  cv::Point2d fov;
  cv::Point2d optical_center;
  cv::Point2d degrees_per_pixel;
};

class AprilTagTracker
{
public:
  AprilTagTracker(raspicam::RaspiCam *camera, boost::mutex *camera_mutex);
  AprilTagTracker(sensor_msgs::CameraInfo camera_info, raspicam::RaspiCam *camera, boost::mutex *camera_mutex);
  ~AprilTagTracker();

  Eigen::Matrix4d getRelativeTransform(const cv::Point2d tagPts[]);
  void drawDetections(cv::Mat *image);
  void getImage();
  void getImage(cv::Mat *image);
  void processImage();
  void adjustServo();
  void outputTimingInfo();
  void calculateTransforms(apriltag_tracker::AprilTagDetectionArray *tag_detection_array);

  cv::Mat *image_gs;
  HostCommLayer::Dynamixel *servo;
  CameraProperties camera_properties;

private:
  // Tag
  TagDetectorParams tag_params;
  TagFamily *tag_family;
  TagDetector *tag_detector;
  TagDetectionArray tag_detections;
  double tag_size; //TODO Create an array of tags, their sizes and their priorities

  // Camera
  boost::mutex *camera_mutex;
  raspicam::RaspiCam *camera;

  // Transform
  Eigen::Matrix4d transform_matrix;
  Eigen::Vector3d rotation_matrix;
  Eigen::Quaternion<double> rotation;
  geometry_msgs::Pose pose;

  // Timing
  ros::Time capture_time;

};

}

#endif //PROJECT_APRILTAG_TRACKER_H
