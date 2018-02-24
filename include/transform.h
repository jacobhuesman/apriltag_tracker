#ifndef PROJECT_TRANSFORM_H
#define PROJECT_TRANSFORM_H

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <boost/thread/thread.hpp>
#include <TagDetection.h>


namespace AprilTagTracker
{

typedef enum
{
  THETA_COMPARE = 1,
  DIST_COMPARE = 2
} COMPARE_TYPEDEF;

class Transform
{
public:
  Transform(TagDetection detection, tf2::Stamped<tf2::Transform> tag_tf, tf2::Stamped<tf2::Transform> servo_tf,
            tf2::Transform map_to_tag_tf, int* compare_mode);
  Transform(TagDetection detection, tf2::Stamped<tf2::Transform> tag_tf, tf2::Stamped<tf2::Transform> servo_tf,
            tf2::Transform map_to_tag_tf);

  tf2::Stamped<tf2::Transform> getTagTf();
  tf2::Stamped<tf2::Transform> getServoTf();
  tf2::Transform getMapToTagTf();
  double getTagTheta();
  cv::Point getDetectionCenter();
  TagDetection getDetection();
  bool operator<(Transform tag_tf);

private:
  tf2::Stamped<tf2::Transform> tag_tf;
  tf2::Stamped<tf2::Transform> servo_tf;
  tf2::Transform map_to_tag_tf;
  TagDetection detection;
  double tag_theta;
  int* compare_mode;
};

}

#endif //PROJECT_TRANSFORM_H
