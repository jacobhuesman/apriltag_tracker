#include <apriltag_tracker.h>

namespace AprilTagTracker
{
  AprilTagTracker::AprilTagTracker(raspicam::RaspiCam *camera, boost::mutex *camera_mutex)
  {
    tag_params.newQuadAlgorithm = true;
    //tag_params.adaptiveThresholdValue = 12; //TODO figure out what this means and parameterize
    //tag_params.adaptiveThresholdValue = 20; //TODO figure out what this means and parameterize
    tag_family = new TagFamily("Tag36h11");
    tag_detector = new TagDetector(*tag_family, tag_params);
    tag_size = .0505;

    servo = new HostCommLayer::Dynamixel(0x11);

    this->camera = camera;
    this->camera_mutex = camera_mutex;
    camera_properties.K.val[0] = 503.382;
    camera_properties.K.val[2] = 319.145;
    camera_properties.K.val[4] = 502.282;
    camera_properties.K.val[5] = 237.571;
    camera_properties.fov.x = 62.2; // TODO measure
    camera_properties.fov.y = 48.8; // TODO measure
    camera_properties.width = camera->getWidth();
    camera_properties.height = camera->getHeight();
    camera_properties.optical_center.x = camera_properties.width / 2;
    camera_properties.optical_center.y = camera_properties.height / 2;
    camera_properties.degrees_per_pixel.x = camera_properties.fov.x / camera_properties.width;
    camera_properties.degrees_per_pixel.y = camera_properties.fov.y / camera_properties.height;

    // TODO what do the scalars do?
    image_gs = new cv::Mat(camera->getHeight(), camera->getWidth(), CV_8UC1, cv::Scalar(69,42,200));
  }

  AprilTagTracker::AprilTagTracker(sensor_msgs::CameraInfo camera_info, raspicam::RaspiCam *camera,
                                   boost::mutex *camera_mutex)
      : AprilTagTracker::AprilTagTracker(camera, camera_mutex)
  {
    this->camera_properties.width = camera_info.width;
    this->camera_properties.height = camera_info.height;
    this->camera_properties.binning_x = camera_info.binning_x;
    this->camera_properties.binning_y = camera_info.binning_y;
    for (int i = 0; i < camera_info.D.size(); i++)
    {
      this->camera_properties.D.push_back(camera_info.D[i]);
    }
    for (int i = 0; i < 9; i++)
    {
      this->camera_properties.K.val[i] = camera_info.K[i];
      this->camera_properties.R.val[i] = camera_info.R[i];
    }
    for (int i = 0; i < 12; i++)
    {
      this->camera_properties.P.val[i] = camera_info.P[i];
    }
  }

  AprilTagTracker::~AprilTagTracker()
  {
    delete image_gs;
    delete servo;
    delete tag_family;
    delete tag_detector;
  }

  Eigen::Matrix4d AprilTagTracker::getRelativeTransform(const cv::Point2d tagPts[])
  {
    std::vector<cv::Point3d> objPts;
    std::vector<cv::Point2d> imgPts;
    double s = tag_size / 2.0;
    objPts.emplace_back(cv::Point3d(-s,-s, 0));
    objPts.emplace_back(cv::Point3d( s,-s, 0));
    objPts.emplace_back(cv::Point3d( s, s, 0));
    objPts.emplace_back(cv::Point3d(-s, s, 0));

    imgPts.push_back(tagPts[0]);
    imgPts.push_back(tagPts[1]);
    imgPts.push_back(tagPts[2]);
    imgPts.push_back(tagPts[3]);

    cv::Mat rvec, tvec;
    // TODO make sure the properties are accurate
    cv::solvePnP(objPts, imgPts, camera_properties.K, camera_properties.D, rvec, tvec);
    cv::Matx33d r;
    cv::Rodrigues(rvec, r);
    Eigen::Matrix3d wRo;
    wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);

    Eigen::Matrix4d T;
    T.topLeftCorner(3,3) = wRo;
    T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
    T.row(3) << 0,0,0,1;

    return T;
  }

  void AprilTagTracker::drawDetections(cv::Mat *image)
  {
    cv::Point2d *p;
    cv::Point2d cxy;
    size_t id;
    for (int i = 0; i < tag_detections.size(); i++)
    {
      p = tag_detections[i].p;
      cxy = tag_detections[i].cxy;
      id = tag_detections[i].id;

      // plot outline
      cv::line(*image, p[0], p[1], cv::Scalar(255,0,0,0) );
      cv::line(*image, p[1], p[2], cv::Scalar(0,255,0,0) );
      cv::line(*image, p[2], p[3], cv::Scalar(0,0,255,0) );
      cv::line(*image, p[3], p[0], cv::Scalar(255,0,255,0) );

      // mark center
      cv::circle(*image, cv::Point2d(cxy.x, cxy.y), 8, cv::Scalar(0,0,255,0), 2);

      // print ID
      std::ostringstream strSt;
      strSt << "#" << id;
      cv::putText(*image, strSt.str(),
                  cv::Point2d(cxy.x + 10, cxy.y + 10),
                  cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
    }
  }

void AprilTagTracker::getImage()
{
  camera_mutex->lock();
  camera->grab();
  capture_time = ros::Time::now();
  camera->retrieve(image_gs->data);
  camera_mutex->unlock();
}

void AprilTagTracker::getImage(cv::Mat *image)
{
  AprilTagTracker::getImage();
  image = image_gs; // Return result
}

void AprilTagTracker::processImage()
{
  tag_detector->process(*image_gs, camera_properties.optical_center, tag_detections);
}

void AprilTagTracker::adjustServo()
{
  // TODO implement using multiple tags
  if (tag_detections.size() == 1)
  {
    double difference = tag_detections[0].cxy.x - camera_properties.width / 2;
    double rotation = camera_properties.degrees_per_pixel.x * difference / servo->resolution;
    uint16_t position;
    servo->getPosition(&position);
    std::cout << "Original position: " << position;
    servo->setPosition((uint16_t)((double)position - rotation));
    std::cout << ", corrected position: " << position - rotation << std::endl;
  }
}

void AprilTagTracker::calculateAprilTagTransforms(apriltag_tracker::AprilTagDetectionArray *tag_detection_array)
{
  for (int i = 0; i < tag_detections.size(); i++)
  {
    // Get transform
    /* Transform */
    Eigen::Matrix4d transform = getRelativeTransform(tag_detections[i].p);
    Eigen::Matrix3d rotation = transform.block(0, 0, 3, 3);
    Eigen::Quaternion<double> rotation_q = Eigen::Quaternion<double>(rotation);

    // Build tag
    geometry_msgs::PoseStamped tag_pose;
    tag_pose.pose.position.x = transform(0, 3);
    tag_pose.pose.position.y = transform(1, 3);
    tag_pose.pose.position.z = transform(2, 3);
    tag_pose.pose.orientation.x = rotation_q.x();
    tag_pose.pose.orientation.y = rotation_q.y();
    tag_pose.pose.orientation.z = rotation_q.z();
    tag_pose.pose.orientation.w = rotation_q.w();
    tag_pose.header.frame_id ="camera";
    tag_pose.header.stamp = capture_time;
    tag_pose.header.seq = 0; // TODO make into sequence

    apriltag_tracker::AprilTagDetection tag_detection;
    tag_detection.pose = tag_pose;
    tag_detection.id = (int)tag_detections[i].id;
    tag_detection.size = tag_size;
    tag_detection_array->detections.push_back(tag_detection);
  }
}

void AprilTagTracker::estimateRobotPose()
{

}

void AprilTagTracker::outputTimingInfo()
{

}


}


