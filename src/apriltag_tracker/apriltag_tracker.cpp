#include <apriltag_tracker/apriltag_tracker.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <apriltag_tracker/errors.h>

namespace apriltag_tracker
{

AprilTagTracker::AprilTagTracker(CameraInfo *camera_info, std::vector<Tag> *tag_info,
                                 AprilTagTrackerConfig *tracker_config, TransformsCache transforms)
{
  this->tracker_config = tracker_config;

  TagDetectorParams tag_params;
  tag_params.newQuadAlgorithm = true;
  //tag_params.adaptiveThresholdValue = 12; //TODO optimize these gains
  //tag_params.adaptiveThresholdValue = 20; //TODO optimize these gains
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
    cv::circle(*image, cv::Point2d(cxy.x, cxy.y), 8, cv::Scalar(128, 128, 128, 0), 2);

    // print ID
    std::ostringstream strSt;
    strSt << "#" << id;
    cv::putText(*image, strSt.str(), cv::Point2d(cxy.x + 10, cxy.y + 10),
                cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(128, 128, 128), 2);
  }
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
  tag_transforms.clear();
  for (int i = 0; i < tag_detections.size(); i++)
  {
    for (int j = 0; j < tag_info->size(); j++)
    {
      if (tag_detections[i].id == (*tag_info)[j].getID())
      {
        // Get transform
        Eigen::Matrix4d transform = getRelativeTransform(tag_detections[i].p, (*tag_info)[j].getSize());
        Eigen::Matrix3d rotation = transform.block(0, 0, 3, 3);
        Eigen::Quaternion<double> rotation_q = Eigen::Quaternion<double>(rotation);

        // Build transform
        tf2::Stamped<tf2::Transform> tag_transform;
        tag_transform.setOrigin(
            tf2::Vector3(transform(0, 3), transform(1, 3), transform(2, 3))); // TODO double check this
        tf2::Quaternion q;
        q.setX(rotation_q.x());
        q.setY(rotation_q.y());
        q.setZ(rotation_q.z());
        q.setW(rotation_q.w());
        tag_transform.setRotation(q);
        tag_transform.stamp_ = this->last_capture_time;

        // Add transform to local tag_transforms object
        geometry_msgs::TransformStamped tag_transform_msg = tf2::toMsg(tag_transform);
        tag_transform_msg.header.seq = this->current_seq;
        tag_transform_msg.header.stamp = this->last_capture_time;
        tag_transform_msg.header.frame_id = ros::this_node::getName() + "_camera_optical";
        tag_transform_msg.child_frame_id = ros::this_node::getName() + "_tag"
                                           + std::to_string((*tag_info)[j].getID()) + "_estimate";

        tag_transforms.push_back(tag_transform_msg);

        // Add transform to global tag_info object
        (*tag_info)[j].addTransform(tag_detections[i], tag_transform, servo_tf, this->current_seq);
      }
    }
  }

  // Try to add filtered transforms
  for (int i = 0; i < tag_info->size(); i++)
  {
    // Try to get median filtered transform
    // TODO fix first
    /*try
    {
      geometry_msgs::TransformStamped tag_transform_msg =
          tf2::toMsg((*tag_info)[i].getMedianFilteredTransform().getTagTf());
      tag_transform_msg.header.seq = this->current_seq;
      tag_transform_msg.header.stamp = this->last_capture_time;
      tag_transform_msg.header.frame_id = "camera_optical";
      tag_transform_msg.child_frame_id = std::string("tag") + std::to_string((*tag_info)[i].getID())
                                         + "_median_filtered_estimate";
      tag_transforms.push_back(tag_transform_msg);
    }
    catch (unable_to_find_transform_error &e) {}*/

    // Try to get average filtered transform
    try
    {
      geometry_msgs::TransformStamped tag_transform_msg =
          tf2::toMsg((*tag_info)[i].getMovingAverageTransform().getTagTf());
      tag_transform_msg.header.seq = this->current_seq;
      tag_transform_msg.header.stamp = this->last_capture_time;
      tag_transform_msg.header.frame_id = "camera_optical";
      tag_transform_msg.child_frame_id = std::string("tag") + std::to_string((*tag_info)[i].getID())
                                         + "_moving_average_estimate";
      tag_transforms.push_back(tag_transform_msg);
    }
    catch (unable_to_find_transform_error &e) {}
  }

  // Try to get the best transform
  try
  {
    Transform tag_transform = getTransform();
    geometry_msgs::TransformStamped tag_transform_msg = tf2::toMsg(tag_transform.getTagTf());
    tag_transform_msg.header.seq = this->current_seq;
    tag_transform_msg.header.stamp = this->last_capture_time;
    tag_transform_msg.header.frame_id = "camera_optical";
    tag_transform_msg.child_frame_id = std::string("tag") + std::to_string(tag_transform.getDetection().id) // TODO more efficient way
                                       + "_trig_estimate";
    tag_transforms.push_back(tag_transform_msg);
  }
  catch (unable_to_find_transform_error &e) {}
}

std::vector<geometry_msgs::TransformStamped> AprilTagTracker::getTagTransforms()
{
  return tag_transforms;
}

Transform AprilTagTracker::performThetaCorrection(Transform tag_a, Transform tag_b,
                                                  tf2::Transform map_to_a, tf2::Transform map_to_b)
{
  tf2::Vector3 point_A = tag_a.getTagTf().getOrigin();
  tf2::Vector3 point_B = tag_b.getTagTf().getOrigin();

  //ROS_INFO("Point A: %f, %f, %f | Point B: %f, %f, %f", point_A.getX(), point_A.getY(), point_A.getZ(),
  //                                                      point_B.getX(), point_B.getY(), point_B.getZ());

  // Find the distances between points (distances labeled as the legs of a triangle opposite each point)
  double a_sq = pow(point_B.getX(), 2.0) + pow(point_B.getZ(), 2.0);
  double b_sq = pow(point_A.getX(), 2.0) + pow(point_A.getZ(), 2.0);

  double b = sqrt(b_sq);
  double c = fabs(map_to_a.getOrigin().getY() - map_to_b.getOrigin().getY());

  if (c < 0.001)
  {
    throw unable_to_find_transform_error("Distance between tags is 0, make sure actual tag locations are specified.");
  }

  double c_sq = pow(c, 2.0);

  // By law of cosines
  double theta = acos((-a_sq + b_sq + c_sq) / (2 * b * c));
  double alpha = atan(point_A.getX()/point_A.getZ());


  //printf("A: %f\n", A);

  tf2::Stamped<tf2::Transform> new_tf = tag_a.getTagTf();
  tf2::Quaternion q1, q2, q3, q4;
  q1.setRPY(0.0, M_PI, M_PI); // TODO combine q1 and q3
  q2.setRPY(0.0, theta, 0.0);
  q3.setRPY(0.0, -alpha, 0.0);
  q4.setRPY(0.0, -M_PI_2, 0.0);

  new_tf.setRotation(q1*q2*q3*q4);
  Transform new_transform(tag_a.getDetection(), new_tf, tag_a.getServoTf(), tag_a.getMapToTagTf());
  double roll, pitch, yaw;
  Transform::getRPY(new_tf.getRotation(), roll, pitch, yaw);
  //ROS_INFO("A: %f | Y %f", theta, pitch);
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
  int tag0 = -1, tag1 = -1, best_tag = 0;
  for (int i = 0; i < tag_info->size(); i++)
  {
    // Ignore stale detections
    if ((*tag_info)[i].getSeq() == current_seq)
    {
      if ((*tag_info)[i].getID() == 0)
      {
        tag0 = i;
      }
      else if ((*tag_info)[i].getID() == 1)
      {
        tag1 = i;
      }
      if ((*tag_info)[i].getPriority() > (*tag_info)[best_tag].getPriority()) // TODO add distance limitations for each tag
      {
        best_tag = i;
      }
    }
  }
  if (tag0 != -1 && tag1 != -1)
  {
    double dt = this->tracker_config->getMaxDt();
    int f_sz = this->tracker_config->getFilterSize();
    return performThetaCorrection((*tag_info)[tag0].getMovingAverageTransform(f_sz, dt),
                                  (*tag_info)[tag1].getMovingAverageTransform(f_sz, dt),
                                  (*tag_info)[tag0].getMapToTagTf(), (*tag_info)[tag1].getMapToTagTf());
  }
  else
  {
    throw unable_to_find_transform_error("Only using moving average transforms currently and can't find two tags");
    // TODO fix first
    //return (*tag_info)[best_tag].getMedianFilteredTransform();
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
                                   * transforms.camera_optical_to_camera_mount
                                   * servo_transform.inverse()
                                   * transforms.dynamixel_to_base_link;
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

AprilTagTrackerConfig::AprilTagTrackerConfig()
{
  this->filter_size = 10;
  this->max_dt = 1.0;

  server = new dynamic_reconfigure::Server<DynamicAprilTagTrackerConfig>;
  dynamic_reconfigure::Server<DynamicAprilTagTrackerConfig>::CallbackType f;
  f = boost::bind(&AprilTagTrackerConfig::reconfigureCallback, this, _1, _2 );
  server->setCallback(f);
}

int AprilTagTrackerConfig::getFilterSize()
{
  return filter_size;
}

double AprilTagTrackerConfig::getMaxDt()
{
  return max_dt;
}

void AprilTagTrackerConfig::reconfigureCallback(apriltag_tracker::DynamicAprilTagTrackerConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure - Filter Size: %i, Max Dt: %f", config.filter_size, config.max_dt);
  this->filter_size = config.filter_size;
  this->max_dt = config.max_dt;
}
}


