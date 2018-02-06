#include <tag.h>

using namespace AprilTagTracker;

Tag::Tag(int id, int priority, double size)
{
  this->id = id;
  //this->frame_id = std::string("tag") + std::to_string(id) + "_estimate";
  this->priority = priority;
  this->size = size;
  this->mutex = new boost::mutex;
  this->seq = 0;

  this->list_size = 5;
}

// TODO add mutex check
void Tag::addTransform(tf2::Stamped<tf2::Transform> tf)
{
  mutex->lock();
  seq++;
  Transform new_transform(tf);
  this->transforms.emplace_front(new_transform);
  if (this->transforms.size() > this->list_size)
  {
    this->transforms.pop_back();
  }
  mutex->unlock();
}

std::vector<Transform> Tag::getTransforms()
{
  return std::vector<Transform>(std::begin(transforms), std::end(transforms));
}

int Tag::getID()
{
  return id;
}

// Not thread safe
/*std::string Tag::getFrameID()
{
  return frame_id;
}*/

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

// TODO handle empty list case
Transform Tag::getMedianFilteredTransform()
{
  /*if (transforms.size() < 3) // TODO this is a bad way of doing this
  {
    tf2::Stamped<tf2::Transform> tf;
    tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    tf2::Quaternion q;
    q.setRPY(0.0,0.0,0.0);
    tf.setRotation(q);
    tf.stamp_ = ros::Time::now();
    Transform zero_transform(tf);
    return zero_transform;
  }
  else
  {*/
    mutex->lock();
    std::list<Transform> transforms_copy(transforms.begin(), transforms.end());
    mutex->unlock();
    transforms_copy.sort();
    auto it = transforms_copy.begin();
    for (int i = 0; i < (transforms_copy.size() / 2); i++)
    {
      it++;
    }
    return it->getTf();
  //}
}

void Tag::setMapToTagTf(tf2::Transform tf) // TODO add unit test
{
  this->map_to_tag_tf = tf;
}

tf2::Transform Tag::getMapToTagTf()
{
  return this->map_to_tag_tf;
}

bool Tag::isReady() // TODO create better metric
{
  if (transforms.size() >= list_size)
  {
    return true;
    /*if ((ros::Time::now() - transforms.end()->getTf().stamp_) < ros::Duration(1.0))
    {
      return true;
    }*/
  }
  return false;
}

