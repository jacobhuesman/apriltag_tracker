#include <transform.h>

using namespace AprilTagTracker;

Transform::Transform(tf2::Stamped<tf2::Transform> tf)
{
  this->tf = tf;

  // TODO this is a little convoluted, is there a more direct way?
  tf2::Matrix3x3 matrix;
  matrix.setRotation(tf.getRotation());
  double tmp1, tmp2;
  matrix.getRPY(tmp1, tmp2, this->theta);
}

tf2::Stamped<tf2::Transform> Transform::getTransform()
{
  return this->tf;
}

double Transform::getTheta()
{
  return this->theta;
}
