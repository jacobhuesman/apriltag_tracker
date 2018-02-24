#include <tag.h>

using namespace AprilTagTracker;

Tag::Tag(int id, int priority, double size)
{
  this->id = id;
  this->priority = priority;
  this->size = size;
  this->mutex = new boost::mutex;
  this->seq = 0;
  this->list_size = 15;
  this->compare_mode = CompareType::distance;
}

// TODO add mutex check
void Tag::addTransform(TagDetection detection, tf2::Stamped<tf2::Transform> tag_tf,
                       tf2::Stamped<tf2::Transform> servo_tf, unsigned int seq)
{
  mutex->lock();
  this->seq = seq;
  Transform new_transform(detection, tag_tf, servo_tf, this->map_to_tag_tf, &(this->compare_mode));
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

Transform Tag::getMostRecentTransform()
{
  if (transforms.size() < 1)
  {
    throw unable_to_find_transform_error("No transforms available for this tag");
  }
  return transforms.front();
}

Transform Tag::getMedianFilteredTransform()
{
  this->compare_mode = CompareType::theta;
  if (transforms.size() < 5)
  {
    throw unable_to_find_transform_error("Median filter not populated");
  }
  mutex->lock();
  std::list<Transform> transforms_copy(transforms.begin(), transforms.end());
  transforms_copy.sort();
  mutex->unlock();
  auto it = transforms_copy.begin();
  for (int i = 0; i < (transforms_copy.size() / 2); i++)
  {
    it++;
  }
  return *it;
}

Transform Tag::getMovingAverageTransform()
{
  if (transforms.size() < list_size)
  {
    throw unable_to_find_transform_error("Moving average filter not populated");
  }
  mutex->lock();
  this->compare_mode = CompareType::distance;
  Transform tag_transform = transforms.front();
  tf2::Stamped<tf2::Transform> tag_tf = tag_transform.getTagTf();
  double x = 0, y = 0, z = 0;
  for (auto it = transforms.begin(); it != transforms.end(); it++)
  {
    tf2::Vector3 origin = it->getTagTf().getOrigin();
    x += origin.getX();
    y += origin.getY();
    z += origin.getZ();
  }
  mutex->unlock();
  double size = transforms.size();
  tag_tf.setOrigin(tf2::Vector3(x / size, y / size, z / size));
  // TODO average the Quaternions?
  return Transform(tag_transform.getDetection(), tag_tf, tag_transform.getServoTf(), tag_transform.getMapToTagTf(),
                   &(this->compare_mode));
}

Transform Tag::getMedianMovingAverageTransform()
{
  if (transforms.size() < list_size)
  {
    throw unable_to_find_transform_error("Moving average filter not populated");
  }
  mutex->lock();
  std::list<Transform> transforms_copy(transforms.begin(), transforms.end());
  mutex->unlock();
  transforms_copy.sort();
  // Don't look at first or last two worst transforms
  auto it_begin = transforms_copy.begin();
  it_begin++;
  it_begin++;
  auto it_end = transforms_copy.end();
  it_end--;
  it_end--;
  Transform tag_transform = transforms_copy.front();
  tf2::Stamped<tf2::Transform> tag_tf = tag_transform.getTagTf();
  double x = 0, y = 0, z = 0;
  for (auto it = it_begin; it != it_end; it++)
  {
    tf2::Vector3 origin = it->getTagTf().getOrigin();
    x += origin.getX();
    y += origin.getY();
    z += origin.getZ();
  }
  double size = transforms.size();
  tag_tf.setOrigin(tf2::Vector3(x / size, y / size, z / size));
  // TODO average the Quaternions?
  return Transform(tag_transform.getDetection(), tag_tf, tag_transform.getServoTf(), tag_transform.getMapToTagTf(),
                   &(this->compare_mode));
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

void Tag::setMapToTagTf(tf2::Transform tf) // TODO add unit test
{
  this->map_to_tag_tf = tf;
}

tf2::Transform Tag::getMapToTagTf()
{
  return this->map_to_tag_tf;
}

cv::Point Tag::getDetectionCenter()
{
  return transforms.front().getDetectionCenter();
}



