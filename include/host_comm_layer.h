#ifndef PROJECT_HOST_COMM_LAYER_H
#define PROJECT_HOST_COMM_LAYER_H

#include <cstdint>
#include <cmath>
#include <boost/thread/mutex.hpp>

#include <mraa.hpp>

#include <comm_layer_defs.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace HostCommLayer
{
  class Dynamixel
  {
  public:
    explicit Dynamixel(uint8_t i2c_address);
    Dynamixel(uint8_t i2c_address, uint8_t i2c_bus);
    ~Dynamixel();

    // Thread-Safe
    tf2::Transform getTransform();
    geometry_msgs::TransformStamped getTransformMsg();
    uint8_t adjustCamera(int16_t velocity);

    // Not Thread-Safe
    void resetI2c();
    uint8_t computeChecksum(CLMessage32 message);
    uint8_t setPosition(uint16_t position);
    uint8_t setVelocity(uint16_t velocity);
    uint8_t setPollingDt(uint16_t polling_dt);
    uint8_t getPositionTx();
    uint8_t getPositionRx(uint16_t *position);
    uint8_t getPosition(uint16_t *position);
    uint8_t getTestMessage(CLMessage32 *test_msg);


    const float resolution = 0.29; // Degrees

  private:
    uint8_t address;
    mraa::I2c *i2c;
    long errors;
    boost::mutex mutex;
    int16_t max_velocity;

    tf2::Transform transform;
    ros::Time stamp;
    unsigned int seq;
    std::string frame_id;
    std::string child_frame_id;
  };
}


#endif //PROJECT_HOST_COMM_LAYER_H
