#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <apriltag_tracker/AprilTagDetection.h>
#include <apriltag_tracker/AprilTagDetectionArray.h>

#include <apriltag_tracker.h>



int main(int argc, char **argv)
{
  // Initialize tracker
  AprilTagTracker::AprilTagTracker tracker;
  tracker.servo->setPosition(512);

  // Initialize ROS
  ros::init(argc, argv, "apriltag_detector");
  ros::NodeHandle nh;
  tf::TransformBroadcaster tf_pub;
  ros::Publisher detections_pub = nh.advertise<apriltag_tracker::AprilTagDetection>("tag_detections", 1);
  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub = it.advertise("tag_detections_image", 1);
  std_msgs::Header image_header;
  image_header.frame_id = "camera";
  cv_bridge::CvImage cv_bridge_image(image_header, sensor_msgs::image_encodings::MONO8, *tracker.image_gs);


  // Note: Entire loop must take less than 33ms
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wmissing-noreturn"
  while(ros::ok())
  {
    // Get image and track
    tracker.getAndProcessImage();
    tracker.adjustServo();

    // Send Transforms
    apriltag_tracker::AprilTagDetectionArray tag_detection_array;
    tracker.populateAprilTagDetectionsArray(&tag_detection_array); // TODO publish this array
    for (int i = 0; i < tag_detection_array.detections.size(); i++)
    {
      tf::Stamped<tf::Transform> tag_transform;
      tf::poseStampedMsgToTF(tag_detection_array.detections[i].pose, tag_transform);
      tf_pub.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, tag_transform.frame_id_,
                                                "tag" + tag_detection_array.detections[i].id));
    }

    // Publish images
    tracker.drawDetections(tracker.image_gs);
    image_pub.publish(cv_bridge_image.toImageMsg());


    // Timing info
    /*//calc_poinsetw(4)t[2] = std::chrono::system_clock::now();
    calc_time[0] = std::chrono::duration_cast<std::chrono::milliseconds>(calc_point[1] - calc_point[0]); // Grab
    calc_time[1] = std::chrono::duration_cast<std::chrono::milliseconds>(calc_point[2] - calc_point[1]); // Retrieve
    calc_time[2] = std::chrono::duration_cast<std::chrono::milliseconds>(calc_point[3] - calc_point[2]); // Grayscale
    calc_time[3] = std::chrono::duration_cast<std::chrono::milliseconds>(calc_point[4] - calc_point[3]); // Process
    calc_time[4] = std::chrono::duration_cast<std::chrono::milliseconds>(calc_point[5] - calc_point[4]); // Get transform
    calc_time[5] = std::chrono::duration_cast<std::chrono::milliseconds>(calc_point[6] - calc_point[5]); // Draw tag on image
    calc_time[6] = std::chrono::duration_cast<std::chrono::milliseconds>(calc_point[7] - calc_point[6]); // Publish image
    calc_time[7] = std::chrono::duration_cast<std::chrono::milliseconds>(calc_point[7] - calc_point[0]); // Total time
    running_count += calc_time[7].count();
    iterations++;


    std::cout << "Grab: "           << std::setw(2) << calc_time[0].count();
  //std::cout << ", retrieve: "     << std::setw(2) << calc_time[1].count();
  //std::cout << ", grayscale: "    << std::setw(2) << calc_time[2].count();
    std::cout << ", process: "      << std::setw(2) << calc_time[3].count();
    std::cout << ", pub_trans: "    << std::setw(2) << calc_time[4].count();
  //std::cout << ", draw: "         << std::setw(2) << calc_time[5].count();
    std::cout << ", pub_img: "      << std::setw(2) << calc_time[6].count();
    std::cout << ", total: "        << std::setw(2) << calc_time[7].count();
    std::cout << ", avg_tot_time: " << std::setw(2) << running_count / iterations;
    if (detections.size() > 0)
    {
      std::cout << ", cxy: " << detections[0].cxy.x << ", " << detections[0].cxy.y;
      std::cout << ", hxy: " << std::setw(3) << detections[0].cxy.x << ", ";
      std::cout << detections[0].cxy.y << std::endl;
    }
    std::cout << std::endl;*/

    //std::string image_path = "/home/nrmc/ws/test_images/image" + std::to_string(iterations) + ".jpg";
    //cv::imwrite(image_path, captured_image_mat);

  }
  #pragma clang diagnostic pop
}

