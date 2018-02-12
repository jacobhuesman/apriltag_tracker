#ifndef PROJECT_TIMER_H
#define PROJECT_TIMER_H

#include <chrono>
#include <vector>
#include <boost/thread.hpp>
#include <apriltag_tracker/ATTLocalTiming.h>

namespace AprilTagTracker
{

class Timer
{
public:
  void start();
  void stop();
  long getTime();

private:
  std::chrono::system_clock::time_point start_time;
  std::chrono::system_clock::time_point end_time;
};

class Timers
{
public:
  Timer image_capture;
  Timer get_image;
  Timer process_image;
  Timer adjust_servo;
  Timer calculate_transforms;
  Timer publish_transforms;
  Timer publish_detections_array;
  Timer publish_plain_image;
  Timer draw_detections;
  Timer publish_detections_image;
  Timer spin;

  long getTotalTime();
  long getProcessingTime();
  apriltag_tracker::ATTLocalTiming getTimingMsg();
};

}

#endif //PROJECT_TIMER_H
