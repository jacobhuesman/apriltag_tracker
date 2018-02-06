#include <tag.h>

using namespace AprilTagTracker;

Tag::Tag(int id, int priority, double size,  boost::mutex *mutex)
{
  this->id = id;
  this->priority = priority;
  this->size = size;
  this->mutex = mutex;
  this->seq = 0;

  this->list_size = 10;
}

// TODO add mutex check
void Tag::addTransform(tf2::Stamped<tf2::Transform> tf)
{
  seq++;
  Transform new_transform(tf);
  this->transforms.emplace_front(new_transform);
  if (this->transforms.size() > this->list_size)
  {
    this->transforms.pop_back();
  }
}

std::vector<Transform> Tag::getTransforms()
{
  return std::vector<Transform>(std::begin(transforms), std::end(transforms));
}

int Tag::getID()
{
  return id;
}

unsigned int Tag::getSeq()
{
  return seq;
}

double Tag::getGoodness()
{
  return goodness;
}

double Tag::getPriority()
{
  return priority;
}

double Tag::getSize()
{
  return size;
}
