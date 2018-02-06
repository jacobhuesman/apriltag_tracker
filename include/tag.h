#ifndef PROJECT_TAG_H
#define PROJECT_TAG_H

#include <list>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <boost/thread/thread.hpp>

#include <transform.h>

namespace AprilTagTracker
{

class Tag
{
public:
  // TODO some of this should be made private
  Tag(int id, int priority, double size, boost::mutex *mutex);

  void addTransform(tf2::Stamped<tf2::Transform> tf);

  int getID();
  unsigned int getSeq();
  double getGoodness();
  double getPriority();
  double getSize();
  std::vector<Transform> getTransforms();


private:
  boost::mutex *mutex;
  int id;
  unsigned int seq;
  double goodness;
  double priority;
  double size;

  int list_size;
  tf2::Transform map_to_tag_tf;

  std::list<AprilTagTracker::Transform> transforms;
};

}

#endif //PROJECT_TAG_H
