#ifndef PROJECT_APRILTAG_TRACKER_H
#define PROJECT_APRILTAG_TRACKER_H

#include <chrono>
#include <boost/thread/thread.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <apriltag_tracker/AprilTagDetection.h>
#include <apriltag_tracker/AprilTagDetectionArray.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>

#include <TagDetector.h>
#include <raspicam/raspicam.h>

#include <host_comm_layer.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera.h>

namespace AprilTagTracker
{

struct TagInfo
{
  boost::mutex *mutex;
  int id;
  unsigned int seq;
  double goodness;
  double priority;
  double size;
  tf2::Transform map_to_tag_tf;
  tf2::Stamped<tf2::Transform> tag_transform;
};



class AprilTagTracker
{
public:
  AprilTagTracker(apriltag_tracker::Camera *camera, HostCommLayer::Dynamixel *servo, std::vector<TagInfo> *tag_info,
                  tf2::Transform camera_optical_to_base_link_tf);
  ~AprilTagTracker();

  Eigen::Matrix4d getRelativeTransform(const cv::Point2d tagPts[], double tag_size);
  void drawDetections();
  void processImage();
  void adjustServo();
  void outputTimingInfo();
  void calculateTransforms(apriltag_tracker::AprilTagDetectionArray *tag_detection_array);
  void estimateRobotPose(geometry_msgs::TransformStamped *pose_estimate_msg);

  HostCommLayer::Dynamixel *servo;  // TODO remove servo dependency
  apriltag_tracker::Camera *camera; // TODO remove camera dependency

private:
  // Tag
  TagDetectorParams tag_params;
  TagFamily *tag_family;
  TagDetector *tag_detector;
  TagDetectionArray tag_detections;
  std::vector<TagInfo> *tag_info;
};

}

#endif //PROJECT_APRILTAG_TRACKER_H
