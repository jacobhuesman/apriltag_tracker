/*Program for the senior design node
1. Captures an image from a USB camera,
2. Pipes it into apriltags.
3. Then finds the transform from the camera to the tag
   and extracts the angle from this transform
4. Tells the servo to move to this angle through
   an overdamped PI compensator (for now,
   will later need to be type 2 system.)
5. Sends transform from tag to camera X,Y,theta
   over CAN in a format
*/

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>

#include <TagDetector.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <chrono> // for cross platform waiting
#include <thread>

#define GAIN1 .82f
#define GAIN2 .23f

Eigen::Matrix4f getRelativeTransform(double tag_size,const cv::Point2f tagPts[], double fx, double fy, double px, double py);

int main()
{
  /* AprilTag Declarations and Initialization */
  TagDetectorParams tag_params; //leave defaults for now
  /*
    tag_params.sigma = ;
    tag_params.segSigma = ;
    tag_params.thetaThresh = ;
    tag_params.magThresh = ;
    tag_params.adaptiveThresholdValue = ;
    tag_params.adaptiveThresholdRadius = ;
  */
  TagFamily tag_family("Tag36h11");
  TagDetector tag_detector(tag_family, tag_params);
  TagDetectionArray detections;

  Eigen::Matrix4f my_t;
  Eigen::Matrix4f camera_in_tag_t;
  Eigen::Vector3f euler_angles;
  float x_pos;
  float y_pos;
  float rotation_angle;

  // These should be loaded from a camera calibration
  float camfx = 730;
  float camfy = 730;
  float campx = 320;
  float campy = 240;

  float tagsize = .0505;

  /* Control System Variables */
  float tagX       = 0;
  float error      = 0;
  float output     = 0;
  float prevOutput = 0;
  float prevError  = 0;

  /* Camera Declarations and Initialization */
  cv::VideoCapture camera(0);
  while(!camera.isOpened())
  {
    std::cout << "Unable to open video0, waiting 5 seconds and trying again..." << std::endl;
    sleep(5);
    camera.open(0);
  }

  cv::Mat captured_image(480, 640, CV_8UC3, cv::Scalar(69,42,200));
  cv::Mat gs_image(480, 640, CV_8UC3, cv::Scalar(69,42,200));

  cv::Point2d optical_center(240, 320);

  // TODO check if camera is good to go
  std::cout << "Warming up camera (2 seconds)" << std::endl;
  cv::waitKey(2000); //let camera warm up
  std::cout << "Camera hot!" << std::endl;

  //init_file(); //for dynamixel


  // For loop timing
  std::chrono::system_clock::time_point calc_start;
  std::chrono::system_clock::time_point calc_end;
  std::chrono::milliseconds calc_time;

  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wmissing-noreturn"
  while(1)
  {
    calc_start = std::chrono::system_clock::now();

    camera >> captured_image;
    cv::cvtColor(captured_image, gs_image, cv::COLOR_BGR2GRAY);

    tag_detector.process(gs_image, optical_center, detections);
    std::cout << detections.size() << " tags detected" << std::endl;

    for (int i = 0; i < detections.size(); i++)
    {

    }

    // Austin's Code
    /*for(size_t index = 0; index < detections.size(); ++index)
    {
      const TagDetection& current_detection = detections[index];

      // Get transform from camera to tag (tag in camera coord)
      my_t = getRelativeTransform(tagsize, current_detection.p, camfx, camfy, campx, campy);

      // Want to center dynamixel/camera on tag so make tag be in center
      tagX = current_detection.cxy.x;
      error= tagX - 160; //positive error means turn CCW, negative means turn CW
      output = GAIN1 * prevOutput + GAIN2 * prevError;
      //set_velocity(output);
      prevOutput = output;
      prevError = error;

      // Need to get inverse (camera in tag coords)
      camera_in_tag_t = my_t.inverse();
      Eigen::Matrix3f wRo = camera_in_tag_t.topLeftCorner(3,3);

      euler_angles = wRo.eulerAngles(1,2,0);
      std::cout <<"Found tag with ID: " <<current_detection.id <<std::endl;
      std::cout << "X: " <<camera_in_tag_t(0, 3)<<std::endl;
      std::cout << "Y: " <<camera_in_tag_t(1, 3)<<std::endl;
      std::cout << "Z: " <<camera_in_tag_t(2, 3)<<std::endl;
      std::cout << "rotAngles: " <<std::endl
                <<euler_angles(0) << std::endl
                <<euler_angles(1) << std::endl
                <<euler_angles(2) << std::endl;
      //std::cout << "angle: " <<rotation_angle<<std::endl;

      //actual X = Z measurement
      //actual Y = X measurement
      //actual theta = first rotAngle
      rotation_angle = euler_angles(0) > M_PI/2 ? euler_angles(0) - M_PI : euler_angles(0);
      x_pos = camera_in_tag_t(2,3);
      y_pos = camera_in_tag_t(0,3);
      std::cout << "Actual Data: " << std::endl
                << "X    : " << x_pos << std::endl
                << "Y    : " << y_pos << std::endl
                << "Theta: " << rotation_angle << std::endl;


      calc_end = std::chrono::system_clock::now();
      calc_time = std::chrono::duration_cast<std::chrono::milliseconds>(calc_end - calc_start);

      std::cout << "Calculation Time: " << calc_time.count() << std::endl;
    }*/
  }
  #pragma clang diagnostic pop
}

Eigen::Matrix4f getRelativeTransform(double tag_size, const cv::Point2f tagPts[], double fx, double fy, double px, double py)
{
  std::vector<cv::Point3f> objPts;
  std::vector<cv::Point2f> imgPts;
  double s = tag_size/2.;
  objPts.push_back(cv::Point3f(-s,-s, 0));
  objPts.push_back(cv::Point3f( s,-s, 0));
  objPts.push_back(cv::Point3f( s, s, 0));
  objPts.push_back(cv::Point3f(-s, s, 0));

  imgPts.push_back(tagPts[0]);
  imgPts.push_back(tagPts[1]);
  imgPts.push_back(tagPts[2]);
  imgPts.push_back(tagPts[3]);

  cv::Mat rvec, tvec;
  cv::Matx33f cameraMatrix(
      fx, 0, px,
      0, fy, py,
      0,  0,  1);
  cv::Vec4f distParam(0,0,0,0); // all 0?
  cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
  cv::Matx33d r;
  cv::Rodrigues(rvec, r);
  Eigen::Matrix3f wRo;
  wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);

  Eigen::Matrix4f T;
  T.topLeftCorner(3,3) = wRo;
  T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
  T.row(3) << 0,0,0,1;

  return T;
}