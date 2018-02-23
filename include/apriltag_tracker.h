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

#include <sensor_msgs/CameraInfo.h>
#include <camera_info.h>
#include <transform.h>
#include <tag.h>
#include <transforms_cache.h>

namespace AprilTagTracker
{

class AprilTagTracker
{
public:
  AprilTagTracker(apriltag_tracker::CameraInfo *camera_info, std::vector<Tag> *tag_info, TransformsCache transforms);
  ~AprilTagTracker();

  Eigen::Matrix4d getRelativeTransform(const cv::Point2d tagPts[], double tag_size);
  void drawDetections(cv::Mat *image);
  void processImage(cv::Mat *image, unsigned int current_seq, ros::Time capture_time,
                    tf2::Stamped<tf2::Transform> servo_tf);
  int16_t getDesiredServoVelocity();
  void updateTags(tf2::Stamped<tf2::Transform> servo_tf);
  std::vector<geometry_msgs::TransformStamped> getTagTransforms();

  Transform performThetaCorrection(Transform tag_a, Transform tag_b, tf2::Transform map_to_a, tf2::Transform map_to_b);
  void estimateRobotPose(geometry_msgs::PoseStamped *pose_estimate_msg);
  Transform getTransform();

  apriltag_tracker::CameraInfo *camera_info;

private:
  // Tag
  TagDetectorParams tag_params;
  TagFamily *tag_family;
  TagDetector *tag_detector;
  TagDetectionArray tag_detections;
  std::vector<geometry_msgs::TransformStamped> tag_transforms;
  std::vector<Tag> *tag_info;
  unsigned int current_seq;
  ros::Time last_capture_time;

  // Transforms
  TransformsCache transforms;
};

}

#endif //PROJECT_APRILTAG_TRACKER_H
