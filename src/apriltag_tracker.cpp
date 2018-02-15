#include <apriltag_tracker.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <errors.h>

namespace AprilTagTracker
{

AprilTagTracker::AprilTagTracker(apriltag_tracker::CameraInfo *camera_info, std::vector<Tag> *tag_info,
                                 TransformsCache transforms)
{
  // TODO use dynamic reconfigure
  tag_params.newQuadAlgorithm = true;
  //tag_params.adaptiveThresholdValue = 12; //TODO figure out what this means and parametrize
  //tag_params.adaptiveThresholdValue = 20; //TODO figure out what this means and parametrize
  tag_family = new TagFamily("Tag36h11");
  tag_detector = new TagDetector(*tag_family, tag_params);

  this->tag_info = tag_info;
  this->camera_info = camera_info;
  this->transforms = transforms;
  this->current_seq = 0;
}

AprilTagTracker::~AprilTagTracker()
{
  delete tag_family;
  delete tag_detector;
}

Eigen::Matrix4d AprilTagTracker::getRelativeTransform(const cv::Point2d tagPts[], double tag_size)
{
  std::vector<cv::Point3d> objPts;
  std::vector<cv::Point2d> imgPts;
  double s = tag_size / 2.0;
  objPts.emplace_back(cv::Point3d(-s, -s, 0));
  objPts.emplace_back(cv::Point3d(s, -s, 0));
  objPts.emplace_back(cv::Point3d(s, s, 0));
  objPts.emplace_back(cv::Point3d(-s, s, 0));

  imgPts.push_back(tagPts[0]);
  imgPts.push_back(tagPts[1]);
  imgPts.push_back(tagPts[2]);
  imgPts.push_back(tagPts[3]);

  cv::Mat rvec, tvec;
  // TODO make sure the properties are accurate
  cv::solvePnP(objPts, imgPts, camera_info->getK(), camera_info->getD(), rvec, tvec);
  cv::Matx33d r;
  cv::Rodrigues(rvec, r);
  Eigen::Matrix3d wRo;
  wRo << r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2);

  Eigen::Matrix4d T;
  T.topLeftCorner(3, 3) = wRo;
  T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  T.row(3) << 0, 0, 0, 1;

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
    cv::line(*image, p[0], p[1], cv::Scalar(255, 0, 0, 0));
    cv::line(*image, p[1], p[2], cv::Scalar(0, 255, 0, 0));
    cv::line(*image, p[2], p[3], cv::Scalar(0, 0, 255, 0));
    cv::line(*image, p[3], p[0], cv::Scalar(255, 0, 255, 0));

    // mark center
    cv::circle(*image, cv::Point2d(cxy.x, cxy.y), 8, cv::Scalar(0, 0, 255, 0), 2);

    // print ID
    std::ostringstream strSt;
    strSt << "#" << id;
    cv::putText(*image, strSt.str(), cv::Point2d(cxy.x + 10, cxy.y + 10),
                cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255));
  }
}

// TODO add unit test
// - Velocity is sane
// - Sequences are correct
int16_t AprilTagTracker::getDesiredServoVelocity()
{
  int best_tag = -1;
  bool updated_tag_available = false;
  for (int i = 0; i < tag_info->size(); i++)
  {
    if ((*tag_info)[i].getSeq() == current_seq)
    {
      updated_tag_available = true;
      best_tag = i;
      break;
    }
  }
  if (!updated_tag_available)
  {
    throw unable_to_find_transform_error("No tag detections available for setting the servo");
  }

  // Find best up to date tag
  for (int i = 0; i < tag_info->size(); i++)
  {
    bool updated = (*tag_info)[i].getSeq() == current_seq;
    bool higher_priority = (*tag_info)[i].getPriority() > (*tag_info)[best_tag].getPriority();
    if (updated && higher_priority)
    {
      best_tag = i;
    }
  }

  // Adjust servo
  const double servo_resolution = 0.29;
  //printf("Detection center: %i\n", (*tag_info)[best_tag].getDetectionCenter().x);
  double difference = ((*tag_info)[best_tag].getDetectionCenter().x - camera_info->getWidth() / 2);
  return -(int16_t) (camera_info->getDegreesPerPixel().x * difference / servo_resolution);
}

void AprilTagTracker::processImage(cv::Mat *image, unsigned int current_seq, ros::Time capture_time,
                                   tf2::Stamped<tf2::Transform> servo_tf)
{
  // TODO there is a full image copy here, see if we can pass by reference
  tag_detector->process(*image, camera_info->getOpticalCenter(), tag_detections);
  this->current_seq = current_seq;
  this->last_capture_time = capture_time;
  updateTags(servo_tf); // TODO might want to call this separately?
}

void AprilTagTracker::updateTags(tf2::Stamped<tf2::Transform> servo_tf)
{
  for (int i = 0; i < tag_detections.size(); i++)
  {
    for (int j = 0; j < tag_info->size(); j++)
    {
      if (tag_detections[i].id == (*tag_info)[j].getID())
      {
        // Get transform
        /* Transform */
        Eigen::Matrix4d transform = getRelativeTransform(tag_detections[i].p, (*tag_info)[j].getSize());
        Eigen::Matrix3d rotation = transform.block(0, 0, 3, 3);
        Eigen::Quaternion<double> rotation_q = Eigen::Quaternion<double>(rotation);

        // Build transform
        tf2::Transform tag_transform;
        tag_transform.setOrigin(
            tf2::Vector3(transform(0, 3), transform(1, 3), transform(2, 3))); // TODO double check this
        tf2::Quaternion q;
        q.setX(rotation_q.x());
        q.setY(rotation_q.y());
        q.setZ(rotation_q.z());
        q.setW(rotation_q.w());
        tag_transform.setRotation(q);
        tf2::Stamped<tf2::Transform> stamped_tag_transform;
        stamped_tag_transform.setData(tag_transform);
        stamped_tag_transform.stamp_ = this->last_capture_time;
        (*tag_info)[j].addTransform(tag_detections[i], stamped_tag_transform, servo_tf, this->current_seq);
      }
    }
  }
}

void AprilTagTracker::fillTagDetectionArray(apriltag_tracker::AprilTagDetectionArray *tag_detection_array)
{
  for (int i = 0; i < tag_detections.size(); i++)
  {
    for (int j = 0; j < tag_info->size(); j++)
    {
      if (tag_detections[i].id == (*tag_info)[j].getID())
      {
        // Transform
        // TODO use previously calculated values from updateTags
        Eigen::Matrix4d transform = getRelativeTransform(tag_detections[i].p, (*tag_info)[j].getSize());
        Eigen::Matrix3d rotation = transform.block(0, 0, 3, 3);
        Eigen::Quaternion<double> rotation_q = Eigen::Quaternion<double>(rotation);

        // Build detection
        apriltag_tracker::AprilTagDetection tag_detection;
        tag_detection.id = (int) tag_detections[i].id;
        tag_detection.size = (*tag_info)[j].getSize();
        tag_detection.transform.header.stamp = this->last_capture_time;
        tag_detection.transform.header.seq = this->current_seq;
        tag_detection.transform.header.frame_id = "camera_optical";
        tag_detection.transform.child_frame_id = std::string("tag") + std::to_string((*tag_info)[j].getID())
                                                 + "_estimate";
        tag_detection.transform.transform.translation.x = transform(0, 3);
        tag_detection.transform.transform.translation.y = transform(1, 3);
        tag_detection.transform.transform.translation.z = transform(2, 3);
        tag_detection.transform.transform.rotation.x = rotation_q.x();
        tag_detection.transform.transform.rotation.y = rotation_q.y();
        tag_detection.transform.transform.rotation.z = rotation_q.z();
        tag_detection.transform.transform.rotation.w = rotation_q.w();

        tag_detection_array->detections.push_back(tag_detection);
      }
    }
  }
}

Transform AprilTagTracker::performThetaCorrection(Transform tag_a, Transform tag_b,
                                                  tf2::Transform map_to_a, tf2::Transform map_to_b)
{
  tf2::Vector3 point_A = tag_a.getTagTf().getOrigin();
  tf2::Vector3 point_B = tag_b.getTagTf().getOrigin();

  // Find the distances between points (distances labeled as the legs of a triangle opposite each point)
  double a_sq = pow(point_B.getX(), 2.0) + pow(point_B.getZ(), 2.0);
  double b_sq = pow(point_A.getX(), 2.0) + pow(point_A.getZ(), 2.0);

  double a = sqrt(a_sq);
  double b = sqrt(b_sq);
  double c = fabs(map_to_a.getOrigin().getY() - map_to_b.getOrigin().getY());

  if (c < 0.001)
  {
    throw unable_to_find_transform_error("Distance between tags is 0, make sure actual tag locations are specified.");
  }

  double c_sq = pow(c, 2.0);

  // By law of cosines
  double A = acos((-a_sq + b_sq + c_sq) / (2 * b * c));

  tf2::Stamped<tf2::Transform> new_tf = tag_a.getTagTf();
  tf2::Quaternion q;
  q.setRPY(0.0, A, 0.0);
  new_tf.setRotation(q);
  Transform new_transform(tag_a.getDetection(), new_tf, tag_a.getServoTf(), tag_a.getMapToTagTf());
  return new_transform;
}

Transform AprilTagTracker::getTransform()
{
  // Make sure we have enough tags to find the transform
  int count = 0;
  for (int i = 0; i < tag_info->size(); i++)
  {
    if ((*tag_info)[i].getSeq() == current_seq)
    {
      count++;
    }
  }
  if (count < 1)
  {
    throw unable_to_find_transform_error("No recent tag transforms available");
  }

  // See if we can use two tags to correct the AprilTag angle, otherwise use the highest priority tag
  int tag1 = -1, tag2 = -1, best_tag = 0;
  for (int i = 0; i < tag_info->size(); i++)
  {
    // Ignore stale detections
    if ((*tag_info)[i].getSeq() == current_seq)
    {
      if ((*tag_info)[i].getID() == 1)
      {
        tag1 = i;
      }
      else if ((*tag_info)[i].getID() == 3)
      {
        tag2 = i;
      }
      if ((*tag_info)[i].getPriority() > (*tag_info)[best_tag].getPriority())
      {
        best_tag = i;
      }
    }
  }
  if (tag1 != -1 && tag2 != -1)
  {
    printf("Performed theta correction \n");
    performThetaCorrection((*tag_info)[tag1].getMostRecentTransform(),
                                  (*tag_info)[tag2].getMostRecentTransform(),
                                  (*tag_info)[tag1].getMapToTagTf(), (*tag_info)[tag2].getMapToTagTf());
    return (*tag_info)[best_tag].getMedianFilteredTransform();
  }
  else
  {
    return (*tag_info)[best_tag].getMedianFilteredTransform();
  }
}

void AprilTagTracker::estimateRobotPose(geometry_msgs::PoseStamped *pose_estimate_msg)
{
  Transform transform = getTransform();
  tf2::Transform map_to_tag_tf = transform.getMapToTagTf();
  tf2::Stamped<tf2::Transform> tag_transform = transform.getTagTf();
  tf2::Stamped<tf2::Transform> servo_transform = transform.getServoTf();

  if ((ros::Time::now() - tag_transform.stamp_) < ros::Duration(0.5))
  {
    tf2::Transform pose_estimate = map_to_tag_tf
                                   * tag_transform.inverse()
                                   * transforms.camera_optical_to_servo_joint
                                   * servo_transform.inverse()
                                   * transforms.servo_base_link_to_base_link;
    pose_estimate_msg->header.frame_id = "map";
    pose_estimate_msg->header.seq = current_seq;
    pose_estimate_msg->header.stamp = tag_transform.stamp_;
    pose_estimate_msg->pose.position.x = pose_estimate.getOrigin().getX();
    pose_estimate_msg->pose.position.y = pose_estimate.getOrigin().getY();
    pose_estimate_msg->pose.position.z = pose_estimate.getOrigin().getY();
    pose_estimate_msg->pose.orientation.x = pose_estimate.getRotation().getX();
    pose_estimate_msg->pose.orientation.y = pose_estimate.getRotation().getY();
    pose_estimate_msg->pose.orientation.z = pose_estimate.getRotation().getZ();
    pose_estimate_msg->pose.orientation.w = pose_estimate.getRotation().getW();
  }
}

}


