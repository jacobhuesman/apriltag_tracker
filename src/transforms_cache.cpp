#include <transforms_cache.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using namespace AprilTagTracker;

void TransformsCache::initializeTagVector(std::vector<Tag> *tag_info)
{
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  for (int i = 0; i < tag_info->size(); i++)
  {
    std::string tag_name = "tag" + std::to_string((*tag_info)[i].getID());
    bool transformed;
    do
    {
      transformed = tfBuffer.canTransform("map", tag_name, ros::Time(0), ros::Duration(1));
      if (!transformed)
      {
        std::stringstream ss;
        ss << "Unable to find transform from " << "map" << " to " << tag_name;
        ROS_WARN("%s", ss.str().c_str());
      }
    } while(ros::ok() && !transformed);
    ROS_INFO("Found transform for tag%i", (*tag_info)[i].getID());

    geometry_msgs::TransformStamped transform_msg;
    transform_msg = tfBuffer.lookupTransform("map", tag_name, ros::Time(0));
    tf2::Transform map_to_tag_tf;
    tf2::fromMsg(transform_msg.transform, map_to_tag_tf);
    (*tag_info)[i].setMapToTagTf(map_to_tag_tf);
  }
}

TransformsCache::TransformsCache(ros::NodeHandle nh)
{
  if (!nh.ok())
  {
    throw unable_to_find_transform_error("Node handle not active!");
  }

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  std::stringstream ss;
  bool transformed = false;
  do
  {
    transformed = tfBuffer.canTransform("camera_optical", "servo_joint", ros::Time(0), ros::Duration(1));
    if (!transformed)
    {
      ss.clear();
      ss << "Unable to find transform from " << "camera_optical" << " to " << "base_link";
      ROS_WARN("%s", ss.str().c_str());
    }
  } while(ros::ok() && !transformed);
  ss.clear();
  ss << "Found transform from " << "camera_optical" << " to " << "servo_joint";
  ROS_INFO("%s", ss.str().c_str());
  geometry_msgs::TransformStamped transform_msg;
  transform_msg = tfBuffer.lookupTransform("camera_optical", "servo_joint", ros::Time(0));
  tf2::fromMsg(transform_msg.transform, this->camera_optical_to_servo_joint);
  do
  {
    transformed = tfBuffer.canTransform("servo_base_link", "base_link", ros::Time(0), ros::Duration(1));
    if (!transformed)
    {
      ss.clear();
      ss << "Unable to find transform from " << "servo_base_link" << " to " << "base_link";
      ROS_WARN("%s", ss.str().c_str());
    }
  } while(ros::ok() && !transformed);
  ss.clear();
  ss << "Found transform from " << "servo_base_link" << " to " << "base_link";
  ROS_INFO("%s", ss.str().c_str());
  transform_msg = tfBuffer.lookupTransform("servo_base_link", "base_link", ros::Time(0));
  tf2::fromMsg(transform_msg.transform, this->servo_base_link_to_base_link);
}