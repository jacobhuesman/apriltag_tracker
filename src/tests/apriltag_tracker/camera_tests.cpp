#include <gtest/gtest.h>
#include <apriltag_tracker/camera.h>
#include <boost/thread.hpp>

using namespace apriltag_tracker;

TEST(AprilTagTrackerCameraTests, SetAndGetSequence)
{
  ros::Time::init();

  boost::mutex *mutex = new boost::mutex;
  CameraInfo *properties = new CameraInfo;
  DummyCamera camera(mutex, properties);

  // Check that it's initialized to zero
  ASSERT_EQ(0, camera.getSeq());

  // Take picture and see if it's incremented
  camera.grabImage();
  ASSERT_EQ(1, camera.getSeq());

  // Check to see that we can set the increment
  properties->image_seq = 2;
  camera.grabImage();
  ASSERT_EQ(3, camera.getSeq());
}

void grabImageThread(DummyCamera *camera)
{
  for (int i = 0; i < 100000; i++)
  {
    camera->grabImage();
  }
}

TEST(AprilTagTrackerCameraTests, CheckSeqThreaded)
{
  ros::Time::init();

  boost::mutex *mutex = new boost::mutex;
  CameraInfo *properties = new CameraInfo;
  properties->image_seq = 0;
  DummyCamera *camera = new DummyCamera(mutex, properties);

  boost::thread thread0(grabImageThread, camera);
  boost::thread thread1(grabImageThread, camera);
  boost::thread thread2(grabImageThread, camera);
  boost::thread thread3(grabImageThread, camera);
  thread0.join();
  thread1.join();
  thread2.join();
  thread3.join();

  ASSERT_EQ(400000, camera->getSeq());

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
