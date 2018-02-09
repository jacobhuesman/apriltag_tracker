#include <transform.h>

using namespace AprilTagTracker;

Transform::Transform(tf2::Stamped<tf2::Transform> tag_tf)
{
  this->tag_tf = tag_tf;

  // TODO this is a little convoluted, is there a more direct way?
  tf2::Matrix3x3 matrix;
  matrix.setRotation(tag_tf.getRotation());
  double tmp1, tmp2;
  matrix.getRPY(tmp1, this->tag_theta, tmp2);
}

tf2::Stamped<tf2::Transform> Transform::getTagTf()
{
  return this->tag_tf;
}

double Transform::getTagTheta()
{
  return this->tag_theta;
}

bool Transform::operator<(Transform tag_tf)
{
  return this->getTagTheta() < tag_tf.getTagTheta();
}
