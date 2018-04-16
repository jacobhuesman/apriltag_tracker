#include <apriltag_tracker/camera.h>

namespace apriltag_tracker
{

CameraMaster::CameraMaster(ros::NodeHandle nh)
{
  camera_mutex = new boost::mutex;
  camera = new raspicam::RaspiCam;
  properties = new CameraInfo;

  setupCamera();

  // TODO update depending on camera and config
  std::string info_path = "package://apriltag_tracker/calibrations/${NAME}.yaml";
  manager_camera_info = new camera_info_manager::CameraInfoManager(nh, "camera", info_path);

  server = new dynamic_reconfigure::Server<DynamicCameraConfig>(nh);
  dynamic_reconfigure::Server<DynamicCameraConfig>::CallbackType f;
  f = boost::bind(&CameraMaster::reconfigureCallback, this, _1, _2 );
  server->setCallback(f);

  properties->image_seq = 0;
  setupProperties();
}

// TODO add setCaptureSize?
void CameraMaster::reconfigureCallback(DynamicCameraConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure - Height: %d,  Width: %d, Brightness: %d, Contrast: %d, Shutter Speed: %d, "
               "FPS: %d, Video Stabilization: %s",
           config.height, config.width, config.brightness, config.contrast,
           config.shutter_speed, config.fps, config.video_stabilization?"True":"False");
  camera_mutex->lock();
  camera->release(); // TODO check to see if releasing the camera is necessary
  camera->setWidth(CameraWidth[config.width]);
  camera->setHeight(CameraHeight[config.height]);
  if (config.shutter_speed != -1)
  {
    camera->setShutterSpeed((unsigned int)config.shutter_speed);
  }
  if (config.brightness != -1)
  {
    camera->setBrightness((unsigned int)config.brightness);
  }
  if (config.contrast != -1)
  {
    camera->setContrast((unsigned int)config.contrast);
  }
  if (config.fps != -1)
  {
    camera->setFrameRate((unsigned int)config.fps);
  }
  camera->setVideoStabilization(config.video_stabilization);
  camera->open();
  while(!camera->isOpened()) // TODO add failure condition
  {
    std::cout << "Unable to open camera, waiting 0.5 seconds and trying again..." << std::endl;
    usleep(500000);
    camera->open();
  }
  setupProperties();
  for (int i = 0; i < cameras.size(); i++)
  {
    cameras[i]->setupCapture();
  }
  camera_mutex->unlock();
}

void CameraMaster::setupCamera()
{
  camera_mutex->lock();
  if (camera->isOpened())
  {
    camera->release();
  }
  camera->setFormat(raspicam::RASPICAM_FORMAT_GRAY);
  camera->setRotation(180);
  camera->open();
  while(!camera->isOpened()) // TODO add failure condition
  {
    std::cout << "Unable to open camera, waiting 0.5 seconds and trying again..." << std::endl;
    usleep(500000);
    camera->open();
  }
  camera_mutex->unlock();
}

void CameraMaster::setupProperties()
{
  // TODO add manager_camera_info->load(new url)
  properties->K.val[0] = 503.382;
  properties->K.val[2] = 319.145;
  properties->K.val[4] = 502.282;
  properties->K.val[5] = 237.571;
  properties->fov.x = 62.2; // TODO measure
  properties->fov.y = 48.8; // TODO measure
  properties->width = camera->getWidth();
  properties->height = camera->getHeight();
  properties->optical_center.x = properties->width / 2;
  properties->optical_center.y = properties->height / 2;
  properties->degrees_per_pixel.x = properties->fov.x / properties->width;
  properties->degrees_per_pixel.y = properties->fov.y / properties->height;

  if (!manager_camera_info->isCalibrated())
  {
    ROS_WARN("Using uncalibrated camera!");
  }
  else
  {
    sensor_msgs::CameraInfo camera_info = manager_camera_info->getCameraInfo();

    this->properties->binning_x = camera_info.binning_x;
    this->properties->binning_y = camera_info.binning_y;
    if (this->properties->D.size() != camera_info.D.size()){
      this->properties->D.resize(camera_info.D.size());
    }
    for (int i = 0; i < camera_info.D.size(); i++)
    {
      this->properties->D[i] = camera_info.D[i];
    }
    for (int i = 0; i < 9; i++)
    {
      this->properties->K.val[i] = camera_info.K[i];
      this->properties->R.val[i] = camera_info.R[i];
    }
    for (int i = 0; i < 12; i++)
    {
      this->properties->P.val[i] = camera_info.P[i];
    }
  }
}

Camera* CameraMaster::generateCameraObject()
{
  cameras.push_back(new RaspiCamera(this->camera, camera_mutex, properties));
  return cameras.back();
}

Camera::Camera(boost::mutex *camera_mutex, CameraInfo *properties)
{
  this->camera_mutex = camera_mutex;
  this->camera_info = properties;

  setupCapture();
}

Camera::~Camera()
{
  delete capture;
}

void Camera::setupCapture()
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "camera_optical"; // TODO change to specific camera
  header.seq = 0;
  cv::Mat *image = new cv::Mat(camera_info->height, camera_info->width, CV_8UC1, cv::Scalar(69,42,200));
  capture = new cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, *image);
}

// TODO add global image_seq
void Camera::grabImage()
{
  camera_mutex->lock();
  _grabImage();
  camera_info->image_seq++;
  capture->header.stamp = ros::Time::now();
  capture->header.seq = camera_info->image_seq;
  _retrieveImage(capture->image.data);
  camera_mutex->unlock();
}

cv::Mat Camera::getImage()
{
  return capture->image;
}

cv::Mat* Camera::getImagePtr()
{
  return &(capture->image);
}

sensor_msgs::ImagePtr Camera::getImageMsg()
{
  return capture->toImageMsg();
}

ros::Time Camera::getCaptureTime()
{
  return capture->header.stamp;
}

unsigned int Camera::getSeq()
{
  return capture->header.seq;
}

CameraInfo *Camera::getCameraInfo()
{
  return camera_info;
}

RaspiCamera::RaspiCamera(raspicam::RaspiCam *camera, boost::mutex *camera_mutex, CameraInfo *properties)
    : Camera(camera_mutex, properties)
{
  this->camera = camera;
}

void RaspiCamera::_grabImage()
{
  camera->grab();
}

void RaspiCamera::_retrieveImage(unsigned char *data)
{
  camera->retrieve(data);
}

}

