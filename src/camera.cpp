#include <camera.h>

namespace apriltag_tracker {

CameraMaster::CameraMaster(ros::NodeHandle nh)
{
  camera_mutex = new boost::mutex;
  camera = new raspicam::RaspiCam;
  properties = new CameraProperties;

  setupCamera();

  // TODO update depending on camera and config
  std::string info_path = "package://apriltag_tracker/calibrations/${NAME}.yaml";
  manager_camera_info = new camera_info_manager::CameraInfoManager(nh, "camera", info_path);

  server = new dynamic_reconfigure::Server<apriltag_tracker::AprilTagTrackerConfig>;
  dynamic_reconfigure::Server<apriltag_tracker::AprilTagTrackerConfig>::CallbackType f;
  f = boost::bind(&CameraMaster::reconfigureCallback, this, _1, _2 );
  server->setCallback(f);

  setupProperties();
}

// TODO add setCaptureSize?
void CameraMaster::reconfigureCallback(apriltag_tracker::AprilTagTrackerConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure - Height: %d,  Width: %d, Brightness: %d, Contrast: %d, Shutter Speed: %d, "
               "Video Stabilization: %s",
           config.height, config.width, config.brightness, config.contrast,
           config.shutter_speed, config.video_stabilization?"True":"False");
  camera_mutex->lock();
  camera->release(); // TODO check to see if releasing the camera is necessary
  camera->setWidth(CameraWidth[config.width]);
  camera->setHeight(CameraHeight[config.height]);
  camera->setShutterSpeed((unsigned int)config.shutter_speed);
  camera->setBrightness((unsigned int)config.brightness);
  camera->setContrast((unsigned int)config.contrast);
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
  camera->setWidth(640);
  camera->setHeight(480);
  camera->setShutterSpeed(8000); // 10000, 8000
  camera->setBrightness(80);  // 70, 80
  camera->setContrast(100);    // 100
  camera->setVideoStabilization(true);
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
  cameras.push_back(new Camera(this->camera, camera_mutex, properties));
  return cameras.back();
}

Camera::Camera(raspicam::RaspiCam *camera, boost::mutex *camera_mutex,
                    CameraProperties *properties)
{
  this->camera = camera;
  this->camera_mutex = camera_mutex;
  this->properties = properties;

  setupCapture();
}

Camera::~Camera()
{
  delete capture;
  delete manager_camera_info;
}

void Camera::setupCapture()
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = "camera_optical"; // TODO change to specific camera
  header.seq = 0;
  cv::Mat *image = new cv::Mat(camera->getHeight(), camera->getWidth(), CV_8UC1, cv::Scalar(69,42,200));
  capture = new cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, *image);
}

// TODO add global seq
void Camera::grabImage()
{
  camera_mutex->lock();
  camera->grab();
  capture->header.stamp = ros::Time::now();
  camera->retrieve(capture->image.data);
  camera_mutex->unlock();
}

std::vector<double> Camera::getD()
{
  return properties->D;
}

cv::Matx33d Camera::getK()
{
  return properties->K;
}

cv::Matx33d Camera::getP()
{
  return properties->P;
}

cv::Matx34d Camera::getR()
{
  return properties->R;
}

cv::Point2d Camera::getOpticalCenter()
{
  return properties->optical_center;
}

cv::Point2d Camera::getFov()
{
  return properties->fov;
}

cv::Point2d Camera::getDegreesPerPixel()
{
  return properties->degrees_per_pixel;
}

uint32_t Camera::getWidth()
{
  return properties->width;
}

uint32_t Camera::getHeight()
{
  return properties->height;
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

}

