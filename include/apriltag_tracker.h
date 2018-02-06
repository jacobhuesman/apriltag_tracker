#ifndef PROJECT_APRILTAG_TRACKER_H
#define PROJECT_APRILTAG_TRACKER_H

#include <chrono>
#include <list>
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
#include <transform.h>
#include <tag.h>

namespace AprilTagTracker
{

struct TransformsCache
{
  tf2::Transform camera_optical_to_servo_joint;
  // Supply dynamic servo_joint to servo_base_link tf
  tf2::Transform servo_base_link_to_base_link;
};


class AprilTagTracker
{
public:
  AprilTagTracker(apriltag_tracker::Camera *camera, HostCommLayer::Dynamixel *servo, std::vector<Tag> *tag_info,
                  TransformsCache transforms);
  ~AprilTagTracker();

  Eigen::Matrix4d getRelativeTransform(const cv::Point2d tagPts[], double tag_size);
  void drawDetections();
  void processImage();
  void adjustServo();
  void outputTimingInfo();
  void updateTags();
  void fillTagDetectionArray(apriltag_tracker::AprilTagDetectionArray *tag_detection_array);
  bool estimateRobotPose(geometry_msgs::PoseStamped *pose_estimate_msg);

  HostCommLayer::Dynamixel *servo;  // TODO remove servo dependency
  apriltag_tracker::Camera *camera; // TODO remove camera dependency

private:
  // Tag
  TagDetectorParams tag_params;
  TagFamily *tag_family;
  TagDetector *tag_detector;
  TagDetectionArray tag_detections;
  std::vector<Tag> *tag_info;

  // Transforms
  TransformsCache transforms;
};

}

#endif //PROJECT_APRILTAG_TRACKER_H
