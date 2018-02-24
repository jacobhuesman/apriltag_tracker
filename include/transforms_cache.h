#ifndef PROJECT_TRANSFORMS_CACHE_H
#define PROJECT_TRANSFORMS_CACHE_H

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tag.h>

namespace AprilTagTracker
{

class TransformsCache
{
public:
  TransformsCache() {};
  TransformsCache(ros::NodeHandle nh); // TODO test with ROS tests
  static void initializeTagVector(std::vector<Tag> *tag_info);

  tf2::Transform camera_optical_to_servo_joint;
  tf2::Transform servo_base_link_to_base_link;
};

}

#endif //PROJECT_TRANSFORMS_CACHE_H
