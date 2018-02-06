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
  Transform(tf2::Stamped<tf2::Transform> tf);

  tf2::Stamped<tf2::Transform> getTransform();
  double getTheta();

private:
  tf2::Stamped<tf2::Transform> tf;
  double theta;
};

}

#endif //PROJECT_TRANSFORM_H
