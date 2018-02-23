#ifndef PROJECT_CAMERA_INFO_H
#define PROJECT_CAMERA_INFO_H

#include <opencv2/opencv.hpp>

namespace apriltag_tracker
{

static unsigned int CameraWidth[5] = {320, 640, 960, 1280, 1600}; // TODO probably come up with cleaner way of doing this
static unsigned int CameraHeight[5] = {240, 480, 720, 960, 1200};

struct CameraInfo
{
  // TODO make these private and add setters
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
  unsigned int image_seq;

  std::vector<double> getD();
  cv::Matx33d getK();
  cv::Matx33d getP();
  cv::Matx34d getR();
  cv::Point2d getOpticalCenter();
  cv::Point2d getFov();
  cv::Point2d getDegreesPerPixel();
  uint32_t getWidth();
  uint32_t getHeight();
  unsigned int getSeq();

};

}

#endif //PROJECT_CAMERA_INFO_H
