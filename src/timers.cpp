#include <timers.h>

void AprilTagTracker::Timer::start()
{
  started = true;
  ready = false;
  start_time = std::chrono::system_clock::now();
}

void AprilTagTracker::Timer::stop()
{
  ready = true;
  end_time = std::chrono::system_clock::now();
}

long AprilTagTracker::Timer::getTime()
{
  if (ready && started)
  {
    return std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  }
  return 0;
}

long AprilTagTracker::Timers::getTotalTime()
{
  long total_time = 0;
  total_time += this->get_image.getTime();
  total_time += this->process_image.getTime();
  total_time += this->adjust_servo.getTime();
  total_time += this->calculate_transforms.getTime();
  total_time += this->publish_transforms.getTime();
  total_time += this->publish_tag_transforms.getTime();
  total_time += this->publish_plain_image.getTime();
  total_time += this->draw_detections.getTime();
  total_time += this->publish_detections_image.getTime();
  total_time += this->spin.getTime();
  return total_time;
}

long AprilTagTracker::Timers::getProcessingTime()
{
  long processing_time = 0;
  processing_time += this->process_image.getTime();
  processing_time += this->adjust_servo.getTime();
  processing_time += this->calculate_transforms.getTime();
  processing_time += this->publish_transforms.getTime();
  processing_time += this->publish_tag_transforms.getTime();
  processing_time += this->publish_plain_image.getTime();
  processing_time += this->draw_detections.getTime();
  processing_time += this->publish_detections_image.getTime();
  processing_time += this->spin.getTime();
  return processing_time;
}

apriltag_tracker::ATTLocalTiming AprilTagTracker::Timers::getTimingMsg()
{
  apriltag_tracker::ATTLocalTiming timing_msg;
  timing_msg.get_image = this->get_image.getTime();
  timing_msg.process_image = this->process_image.getTime();
  timing_msg.adjust_servo = this->adjust_servo.getTime();
  timing_msg.calculate_transforms = this->calculate_transforms.getTime();
  timing_msg.publish_transforms = this->publish_transforms.getTime();
  timing_msg.publish_tag_transforms = this->publish_tag_transforms.getTime();
  timing_msg.publish_plain_image = this->publish_plain_image.getTime();
  timing_msg.draw_detections = this->draw_detections.getTime();
  timing_msg.publish_detections_image = this->publish_detections_image.getTime();
  timing_msg.spin = this->spin.getTime();
  timing_msg.total_time = this->getTotalTime();
  timing_msg.processing_time = this->getProcessingTime();
  return timing_msg;
}
