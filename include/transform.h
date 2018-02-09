#ifndef PROJECT_TRANSFORM_H
#define PROJECT_TRANSFORM_H

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <boost/thread/thread.hpp>

namespace AprilTagTracker
{

class Transform
{
public:
  Transform(tf2::Stamped<tf2::Transform> tag_tf);
  //Transform(tf2::Stamped<tf2::Transform> tag_tf, tf2::Stamped<tf2::Transform> servo_tf);

  tf2::Stamped<tf2::Transform> getTagTf();
  double getTagTheta();
  bool operator<(Transform tag_tf);

private:
  tf2::Stamped<tf2::Transform> tag_tf;
  double tag_theta;
};

}

#endif //PROJECT_TRANSFORM_H
