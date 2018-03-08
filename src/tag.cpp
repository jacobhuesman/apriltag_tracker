#include <tag.h>

using namespace apriltag_tracker;

Tag::Tag(int id, int priority, double size)
{
  this->id = id;
  this->priority = priority;
  this->size = size;
  this->mutex = new boost::mutex;
  this->seq = 0;
  this->list_size = 20;
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

Transform Tag::getMovingAverageTransform(int n_tf, double max_dt)
{
  mutex->lock();
  // Flush old detections
  auto it = transforms.begin();
  int good_tfs = 0;
  bool clean = false;
  for (int i = 0; i < transforms.size(); i++, it++)
  {
    ros::Duration time_diff(ros::Time::now() - it->getTagTf().stamp_);
    if (time_diff > ros::Duration(max_dt))
    {
      clean = true;
      good_tfs = i;
      break;
    }
  }
  if (clean)
  {
    int size = transforms.size();
    for (int i = 0; (i + good_tfs) < size; i++)
    {
      transforms.pop_back();
    }
  }

  if (transforms.size() < n_tf)
  {
    mutex->unlock();
    throw unable_to_find_transform_error("Moving average filter not populated");
  }
  this->compare_mode = CompareType::distance;
  it = transforms.begin();
  if (n_tf <= 2)
  {
    // TODO add moving average filter for servo tfs
    for (int i = 0; i < n_tf / 2; it++, i++); // Pick middle one so servo tf doesn't lead quite so badly
  }
  else
  {
    it++; // TODO figure out what element this is actually grabbing...
  }
  Transform tag_transform = *it;
  tf2::Stamped<tf2::Transform> tag_tf = tag_transform.getTagTf();
  it = transforms.begin();
  double x = 0, y = 0, z = 0;
  for (int i = 0; i < n_tf; it++, i++)
  {
    tf2::Vector3 origin = it->getTagTf().getOrigin();
    x += origin.getX();
    y += origin.getY();
    z += origin.getZ();
  }
  mutex->unlock();
  tag_tf.setOrigin(tf2::Vector3(x / n_tf, y / n_tf, z / n_tf));
  // TODO average the Quaternions?
  return Transform(tag_transform.getDetection(), tag_tf, tag_transform.getServoTf(), tag_transform.getMapToTagTf(),
                   &(this->compare_mode));
}

Transform Tag::getMovingAverageTransform(int n_tf)
{
  return Tag::getMovingAverageTransform(n_tf, 1.0);
}

Transform Tag::getMovingAverageTransform()
{
  return Tag::getMovingAverageTransform(this->list_size);
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

double Tag::getAngleFromCenter(int n_tf)
{
  tf2::Vector3 origin = getMovingAverageTransform(n_tf).getTagTf().getOrigin();
  return atan(origin.getX() / origin.getZ());
}

double Tag::getAngleFromCenter()
{
  return getAngleFromCenter(5);
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





