#ifndef APRILTAG_TRACKER_CAMERA_H
#define APRILTAG_TRACKER_CAMERA_H

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
#include <apriltag_tracker/camera_info.h>

namespace apriltag_tracker
{

class Camera
{
public:
  Camera(boost::mutex *camera_mutex, CameraInfo *properties);
  ~Camera();

  void grabImage();
  sensor_msgs::ImagePtr getImageMsg();
  ros::Time getCaptureTime();
  cv::Mat getImage();
  cv::Mat* getImagePtr();
  unsigned int getSeq();
  CameraInfo* getCameraInfo();
  void setupCapture();

private:
  virtual void _grabImage() = 0;
  virtual void _retrieveImage(unsigned char *data) = 0;

  boost::mutex *camera_mutex;
  CameraInfo *camera_info; // Do not edit in this class
  cv_bridge::CvImage *capture;
};

class RaspiCamera : public Camera
{
public:
  RaspiCamera(raspicam::RaspiCam *camera, boost::mutex *camera_mutex, CameraInfo *properties);

private:
  void _grabImage();
  void _retrieveImage(unsigned char *data);

  raspicam::RaspiCam *camera; // Shared object
};

class DummyCamera : public Camera
{
public:
  DummyCamera(boost::mutex *camera_mutex, CameraInfo *properties) : Camera(camera_mutex, properties) {};

private:
  void _grabImage() {};
  void _retrieveImage(unsigned char *data) {};

  raspicam::RaspiCam *camera; // Shared object
};

class CameraMaster
{
public:
  CameraMaster(ros::NodeHandle nh);
  Camera* generateCameraObject();

private:
  raspicam::RaspiCam *camera;
  boost::mutex *camera_mutex;
  CameraInfo *properties;
  std::vector<apriltag_tracker::Camera*> cameras;

  camera_info_manager::CameraInfoManager *manager_camera_info;
  dynamic_reconfigure::Server<apriltag_tracker::DynamicCameraConfig> *server;

  void setupCamera();
  void setupProperties();
  void reconfigureCallback(apriltag_tracker::DynamicCameraConfig &config, uint32_t level);
};

}

#endif //APRILTAG_TRACKER_CAMERA_H
