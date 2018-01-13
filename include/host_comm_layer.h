#ifndef PROJECT_HOST_COMM_LAYER_H
#define PROJECT_HOST_COMM_LAYER_H

#include <cstdint>
#include <cmath>

#include <mraa.hpp>

#include <comm_layer_defs.h>

namespace HostCommLayer
{
  class Dynamixel
  {
  public:
    explicit Dynamixel(uint8_t i2c_address);
    Dynamixel(uint8_t i2c_address, uint8_t i2c_bus);
    ~Dynamixel();

    uint8_t computeChecksum(CLMessage32 message);
    uint8_t setPosition(uint16_t position);
    uint8_t getPositionTx();
    uint8_t getPositionRx(uint16_t *position);
    uint8_t getPosition(uint16_t *position);
    uint8_t getTestMessage(CLMessage32 *test_msg);

    const float resolution = 0.29; // Degrees

  private:
    uint8_t address;
    mraa::I2c *i2c;
    long errors;
  };
}


#endif //PROJECT_HOST_COMM_LAYER_H
