
#include <camera.h>

namespace apriltag_tracker
{

std::vector<double> CameraInfo::getD()
{
  return D;
}

cv::Matx33d CameraInfo::getK()
{
  return K;
}

cv::Matx33d CameraInfo::getP()
{
  return P;
}

cv::Matx34d CameraInfo::getR()
{
  return R;
}

cv::Point2d CameraInfo::getOpticalCenter()
{
  return optical_center;
}

cv::Point2d CameraInfo::getFov()
{
  return fov;
}

cv::Point2d CameraInfo::getDegreesPerPixel()
{
  return degrees_per_pixel;
}

uint32_t CameraInfo::getWidth()
{
  return width;
}

uint32_t CameraInfo::getHeight()
{
  return height;
}

unsigned int CameraInfo::getSeq()
{
  return image_seq;
}


}

