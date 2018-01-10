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
#include <string>

#include <TagDetector.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <chrono> // for cross platform waiting
#include <thread>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <raspicam/raspicam.h>


typedef struct {
  float fx;
  float fy;
  float px;
  float py;
} CameraConfig;

Eigen::Matrix4f getRelativeTransform(CameraConfig cfg, double tag_size, const cv::Point2f tagPts[]);
void draw(cv::Mat& image, const cv::Point2f p[], at::Point cxy, size_t id);


int main(int argc, char **argv)
{
  /*
   * Declarations and Configuration
   */
  /* AprilTag */
  TagDetectorParams tag_params; //leave defaults for now
  // TODO check out other tag params
  tag_params.newQuadAlgorithm = true;
  tag_params.adaptiveThresholdValue = 12; //TODO figure out what this means

  TagFamily tag_family("Tag36h11");
  TagDetector tag_detector(tag_family, tag_params);
  TagDetectionArray detections;

  Eigen::Matrix4f my_t;
  Eigen::Matrix4f camera_in_tag_t;
  Eigen::Vector3f euler_angles;

  // These should be loaded from a camera calibration
  CameraConfig cam0_config;
  cam0_config.fx = 730;
  cam0_config.fy = 730;
  cam0_config.px = 320;
  cam0_config.py = 240;

  float tag_size = .0505;


  /* Camera */
  //cv::VideoCapture camera(0);
  raspicam::RaspiCam camera;
  camera.setWidth(640);
  camera.setHeight(480);
  camera.setFormat(raspicam::RASPICAM_FORMAT_GRAY);
  camera.setRotation(180);

  while(!camera.isOpened())
  {
    std::cout << "Unable to open camera, waiting 1 second and trying again..." << std::endl;
    sleep(1);
    camera.open();
    //camera.open(0);
  }
  int frame_width =  (int)camera.getWidth();
  int frame_height = (int)camera.getHeight();
  double camera_fps = camera.getFrameRate();
  std::cout << "Frame width: " << frame_width << ", frame height: " << frame_height;
  std::cout << ", fps: " << camera_fps << std::endl;

  cv::Mat gs_image(frame_height, frame_width, CV_8UC1, cv::Scalar(69,42,200));
  cv::Point2d optical_center(frame_height/2, frame_width/2);


  /* ROS */
  ros::init(argc, argv, "apriltag_detector");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub = it.advertise("tag_detections_image", 1);

  std_msgs::Header image_header;
  image_header.frame_id = "camera";
  //cv_bridge::CvImage captured_img(image_header, sensor_msgs::image_encodings::BGR8, captured_image_mat);
  cv_bridge::CvImage captured_img(image_header, sensor_msgs::image_encodings::MONO8, gs_image);


  /* Timing Info */
  std::chrono::system_clock::time_point calc_point[10];
  std::chrono::milliseconds calc_time[10];
  long running_count = 0;
  long iterations = 0;

  /*
   * Start
   */
  // TODO check if camera is good to go
  //std::cout << "Warming up camera (2 seconds)" << std::endl;
  //cv::waitKey(2000); //let camera warm up
  //std::cout << "Camera hot!" << std::endl;

  // Note: Entire loop must take less than 33ms
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wmissing-noreturn"
  while(ros::ok())
  {

    // Grab image
    calc_point[0] = std::chrono::system_clock::now();
    //camera >> captured_img.image;
    camera.grab();
    calc_point[1] = std::chrono::system_clock::now();
    camera.retrieve(captured_img.image.data);
    calc_point[2] = std::chrono::system_clock::now();
    //cv::cvtColor(captured_img.image, gs_image, cv::COLOR_BGR2GRAY);
    calc_point[3] = std::chrono::system_clock::now();

    // Process image
    tag_detector.process(gs_image, optical_center, detections);
    calc_point[4] = std::chrono::system_clock::now();

    // Output data
    //std::cout << detections.size() << " tags detected" << std::endl;
    //TODO Add tag sizes
    for (int i = 0; i < detections.size(); i++)
    {
      draw(captured_img.image, detections[i].p, detections[i].cxy, detections[i].id);
    }
    //getRelativeTransform(cam0_config, tag_size, detections[i].p);


    calc_point[5] = std::chrono::system_clock::now();
    image_pub.publish(captured_img.toImageMsg());
    calc_point[6] = std::chrono::system_clock::now();

    //calc_point[2] = std::chrono::system_clock::now();
    calc_time[0] = std::chrono::duration_cast<std::chrono::milliseconds>(calc_point[1] - calc_point[0]);
    calc_time[1] = std::chrono::duration_cast<std::chrono::milliseconds>(calc_point[2] - calc_point[1]);
    calc_time[2] = std::chrono::duration_cast<std::chrono::milliseconds>(calc_point[3] - calc_point[2]);
    calc_time[3] = std::chrono::duration_cast<std::chrono::milliseconds>(calc_point[4] - calc_point[3]);
    calc_time[4] = std::chrono::duration_cast<std::chrono::milliseconds>(calc_point[5] - calc_point[4]);
    calc_time[5] = std::chrono::duration_cast<std::chrono::milliseconds>(calc_point[6] - calc_point[5]);
    calc_time[6] = std::chrono::duration_cast<std::chrono::milliseconds>(calc_point[6] - calc_point[0]);



    running_count += calc_time[6].count();
    iterations++;

    std::cout << "Grab: " << calc_time[0].count() << ", retrieve: " << calc_time[1].count();
    std::cout << ", grayscale: " << calc_time[2].count() << ", process: " << calc_time[3].count();
    std::cout << ", publish: " << calc_time[5].count() << ", total: " << calc_time[6].count();
    std::cout << ", draw: " << calc_time[4].count();
    std::cout << ", average total time: " << running_count / iterations << std::endl;

    //std::string image_path = "/home/nrmc/ws/test_images/image" + std::to_string(iterations) + ".jpg";
    //cv::imwrite(image_path, captured_image_mat);

  }
  #pragma clang diagnostic pop
}

Eigen::Matrix4f getRelativeTransform(CameraConfig cfg, double tag_size, const cv::Point2f tagPts[])
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
      cfg.fx,      0, cfg.px,
      0,      cfg.fy, cfg.py,
      0,           0,      1);
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

void draw(cv::Mat& image, const cv::Point2f p[], at::Point cxy, size_t id)
{
  // plot outline
  cv::line(image, p[0], p[1], cv::Scalar(255,0,0,0) );
  cv::line(image, p[1], p[2], cv::Scalar(0,255,0,0) );
  cv::line(image, p[2], p[3], cv::Scalar(0,0,255,0) );
  cv::line(image, p[3], p[0], cv::Scalar(255,0,255,0) );

  // mark center
  cv::circle(image, cv::Point2f(cxy.x, cxy.y), 8, cv::Scalar(0,0,255,0), 2);

  // print ID
  std::ostringstream strSt;
  strSt << "#" << id;
  cv::putText(image, strSt.str(),
              cv::Point2f(cxy.x + 10, cxy.y + 10),
              cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0,0,255));
}