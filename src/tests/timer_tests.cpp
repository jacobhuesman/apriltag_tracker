#include <gtest/gtest.h>
#include <timers.h>

using namespace apriltag_tracker;

TEST(AprilTagTrackerTimerTests, DefaultTimer)
{
  Timer timer;
  ASSERT_EQ(timer.getTime(), 0);
}

TEST(AprilTagTrackerTimerTests, Timer1000us)
{
  Timer timer;
  timer.start();
  usleep(1000);
  timer.stop();

  ASSERT_GT(timer.getTime(), 800);
  ASSERT_LT(timer.getTime(), 5000);
}

TEST(AprilTagTrackerTimerTests, TimerNoStart)
{
  Timer timer;
  timer.stop();

  ASSERT_EQ(timer.getTime(), 0);
}

TEST(AprilTagTrackerTimerTests, TimerNoStop)
{
  Timer timer;
  timer.start();

  ASSERT_EQ(timer.getTime(), 0);
}

TEST(AprilTagTrackerTimerTests, TimerNoStopOrStart)
{
  Timer timer;

  ASSERT_EQ(timer.getTime(), 0);
}

TEST(AprilTagTrackerTimerTests, Timers)
{
  Timers timer;
  timer.get_image.start();                usleep( 1000); timer.get_image.stop();
  timer.process_image.start();            usleep( 2000); timer.process_image.stop();
  timer.adjust_servo.start();             usleep( 3000); timer.adjust_servo.stop();
  timer.calculate_transforms.start();     usleep( 4000); timer.calculate_transforms.stop();
  timer.publish_tag_transforms.start(); usleep( 5000); timer.publish_tag_transforms.stop();
  timer.publish_transforms.start();       usleep( 6000); timer.publish_transforms.stop();
  timer.publish_plain_image.start();      usleep( 7000); timer.publish_plain_image.stop();
  timer.draw_detections.start();          usleep( 8000); timer.draw_detections.stop();
  timer.publish_detections_image.start(); usleep( 9000); timer.publish_detections_image.stop();
  timer.spin.start();                     usleep(10000); timer.spin.stop();

  ASSERT_GT(timer.get_image.getTime(),                  800);
  ASSERT_LT(timer.get_image.getTime(),                 1200);
  ASSERT_GT(timer.process_image.getTime(),             1800);
  ASSERT_LT(timer.process_image.getTime(),             2200);
  ASSERT_GT(timer.adjust_servo.getTime(),              2800);
  ASSERT_LT(timer.adjust_servo.getTime(),              3200);
  ASSERT_GT(timer.calculate_transforms.getTime(),      3800);
  ASSERT_LT(timer.calculate_transforms.getTime(),      4200);
  ASSERT_GT(timer.publish_tag_transforms.getTime(),  4800);
  ASSERT_LT(timer.publish_tag_transforms.getTime(),  5200);
  ASSERT_GT(timer.publish_transforms.getTime(),        5800);
  ASSERT_LT(timer.publish_transforms.getTime(),        6200);
  ASSERT_GT(timer.publish_plain_image.getTime(),       6800);
  ASSERT_LT(timer.publish_plain_image.getTime(),       7200);
  ASSERT_GT(timer.draw_detections.getTime(),           7800);
  ASSERT_LT(timer.draw_detections.getTime(),           8200);
  ASSERT_GT(timer.publish_detections_image.getTime(),  8800);
  ASSERT_LT(timer.publish_detections_image.getTime(),  9200);
  ASSERT_GT(timer.spin.getTime(),                      9800);
  ASSERT_LT(timer.spin.getTime(),                     10200);
  ASSERT_GT(timer.getTotalTime(),                     54000);
  ASSERT_LT(timer.getTotalTime(),                     56000);
}

TEST(AprilTagTrackerTimerTests, TimersMsg)
{
  Timers timer;
  timer.get_image.start();                usleep( 1000); timer.get_image.stop();
  timer.process_image.start();            usleep( 2000); timer.process_image.stop();
  timer.adjust_servo.start();             usleep( 3000); timer.adjust_servo.stop();
  timer.calculate_transforms.start();     usleep( 4000); timer.calculate_transforms.stop();
  timer.publish_tag_transforms.start();   usleep( 5000); timer.publish_tag_transforms.stop();
  timer.publish_transforms.start();       usleep( 6000); timer.publish_transforms.stop();
  timer.publish_plain_image.start();      usleep( 7000); timer.publish_plain_image.stop();
  timer.draw_detections.start();          usleep( 8000); timer.draw_detections.stop();
  timer.publish_detections_image.start(); usleep( 9000); timer.publish_detections_image.stop();
  timer.spin.start();                     usleep(10000); timer.spin.stop();
  ATTLocalTiming timing_msg = timer.getTimingMsg();

  ASSERT_GT(timing_msg.get_image,                  800);
  ASSERT_LT(timing_msg.get_image,                 1200);
  ASSERT_GT(timing_msg.process_image,             1800);
  ASSERT_LT(timing_msg.process_image,             2200);
  ASSERT_GT(timing_msg.adjust_servo,              2800);
  ASSERT_LT(timing_msg.adjust_servo,              3200);
  ASSERT_GT(timing_msg.calculate_transforms,      3800);
  ASSERT_LT(timing_msg.calculate_transforms,      4200);
  ASSERT_GT(timing_msg.publish_tag_transforms,    4800);
  ASSERT_LT(timing_msg.publish_tag_transforms,    5200);
  ASSERT_GT(timing_msg.publish_transforms,        5800);
  ASSERT_LT(timing_msg.publish_transforms,        6200);
  ASSERT_GT(timing_msg.publish_plain_image,       6800);
  ASSERT_LT(timing_msg.publish_plain_image,       7200);
  ASSERT_GT(timing_msg.draw_detections,           7800);
  ASSERT_LT(timing_msg.draw_detections,           8200);
  ASSERT_GT(timing_msg.publish_detections_image,  8800);
  ASSERT_LT(timing_msg.publish_detections_image,  9200);
  ASSERT_GT(timing_msg.spin,                      9800);
  ASSERT_LT(timing_msg.spin,                     10200);
  ASSERT_GT(timing_msg.total_time,               52000);
  ASSERT_LT(timing_msg.total_time,               58000);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
