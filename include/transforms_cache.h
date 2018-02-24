#ifndef PROJECT_TRANSFORMS_CACHE_H
#define PROJECT_TRANSFORMS_CACHE_H

#include <tf2/transform_datatypes.h>

namespace AprilTagTracker
{

struct TransformsCache
{
  tf2::Transform camera_optical_to_servo_joint;
  tf2::Transform servo_base_link_to_base_link;
};

}

#endif //PROJECT_TRANSFORMS_CACHE_H
