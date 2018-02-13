#ifndef PROJECT_APRILTAG_TRACKER_CAMERA_H
#define PROJECT_APRILTAG_TRACKER_CAMERA_H

#include <boost/thread/thread.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <raspicam/raspicam.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <camera_info_manager/camera_info_manager.h>
#include <apriltag_tracker/DynamicCameraConfig.h>
#include <dynamic_reconfigure/server.h>



namespace apriltag_tracker {

static unsigned int CameraWidth[4] = {320, 640, 960, 1280}; // TODO probably come up with cleaner way of doing this
static unsigned int CameraHeight[4] = {240, 480, 720, 960};

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
  unsigned int seq;
};

class Camera
{
public:
  Camera(raspicam::RaspiCam *camera, boost::mutex *camera_mutex,
         CameraProperties *properties);
  ~Camera();
  void grabImage();

  // Getters
  std::vector<double> getD();
  cv::Matx33d getK();
  cv::Matx33d getP();
  cv::Matx34d getR();
  cv::Point2d getOpticalCenter();
  cv::Point2d getFov();
  cv::Point2d getDegreesPerPixel();
  uint32_t getWidth();
  uint32_t getHeight();
  sensor_msgs::ImagePtr getImageMsg();
  ros::Time getCaptureTime();
  cv::Mat getImage();
  cv::Mat* getImagePtr();
  unsigned int getSeq();

  void setupCapture();


private:
  // Shared objects
  raspicam::RaspiCam *camera;
  boost::mutex *camera_mutex;
  CameraProperties *properties; // Do not edit in this class
  camera_info_manager::CameraInfoManager *manager_camera_info;

  // Private objects
  cv_bridge::CvImage *capture;
};


class CameraMaster
{
public:
  CameraMaster(ros::NodeHandle nh);
  Camera* generateCameraObject();

private:
  raspicam::RaspiCam *camera;
  boost::mutex *camera_mutex;
  CameraProperties *properties;
  std::vector<apriltag_tracker::Camera*> cameras;

  camera_info_manager::CameraInfoManager *manager_camera_info;
  dynamic_reconfigure::Server<apriltag_tracker::DynamicCameraConfig> *server;

  void setupCamera();
  void setupProperties();
  void reconfigureCallback(apriltag_tracker::DynamicCameraConfig &config, uint32_t level);
};

}

#endif //PROJECT_APRILTAG_TRACKER_CAMERA_H
