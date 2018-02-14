#ifndef PROJECT_TRANSFORM_H
#define PROJECT_TRANSFORM_H

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <boost/thread/thread.hpp>
#include <TagDetection.h>


namespace AprilTagTracker
{

class Transform
{
public:
  Transform(TagDetection detection, tf2::Stamped<tf2::Transform> tag_tf, tf2::Stamped<tf2::Transform> servo_tf);

  tf2::Stamped<tf2::Transform> getTagTf();
  tf2::Stamped<tf2::Transform> getServoTf();
  double getTagTheta();
  cv::Point getDetectionCenter();
  bool operator<(Transform tag_tf);

private:
  tf2::Stamped<tf2::Transform> tag_tf;
  tf2::Stamped<tf2::Transform> servo_tf;
  TagDetection detection;

  double tag_theta;
};

}

#endif //PROJECT_TRANSFORM_H
