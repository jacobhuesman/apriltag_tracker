#include <host_comm_layer.h>
#include <comm_layer_defs.h>
#include <iostream>

namespace HostCommLayer
{

Dynamixel::Dynamixel(uint8_t i2c_address)
{
  i2c = new mraa::I2c(0);
  this->address = i2c_address;

  seq = 0;
  transform.setOrigin(tf2::Vector3(24.15e-3, 0.0, 32.5e-3));
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  frame_id = "servo_base_link";
  child_frame_id = "servo_joint";
}

void Dynamixel::resetI2c()
{
  delete i2c;
  usleep(100);
  i2c = new mraa::I2c(0);
}

Dynamixel::Dynamixel(uint8_t i2c_address, uint8_t i2c_bus)
{
  i2c = new mraa::I2c(i2c_bus);
  this->address = i2c_address;
}

Dynamixel::~Dynamixel()
{
  delete i2c;
}

uint8_t Dynamixel::computeChecksum(CLMessage32 message)
{
  uint8_t checksum = 0;
  for (int i = 0; i < 3; i++)
  {
    checksum += message.data8[i];
  }
  return checksum;
}

//TODO add error checking for both setPosition and getPosition
uint8_t Dynamixel::setPosition(uint16_t position)
{
  CLMessage32 message;
  i2c->address(address);
  message.cl.instruction = DYN_SET_POSITION;
  message.cl.data = position;
  message.cl.checksum = computeChecksum(message);
  if (i2c->write(message.data8, 4) != MRAA_SUCCESS)
  {
    return CL_TX_ERROR;
  }
  return CL_OK;
}

uint8_t Dynamixel::getPositionTx()
{
  CLMessage32 message;
  i2c->address(address);
  message.cl.instruction = DYN_GET_POSITION_INSTRUCTION;
  message.cl.data = 0;
  message.cl.checksum = computeChecksum(message);
  if (i2c->write(message.data8, 4) != MRAA_SUCCESS)
  {
    return CL_TX_ERROR;
  }
  return CL_OK;
}

uint8_t Dynamixel::getPositionRx(uint16_t *position)
{
  CLMessage32 message;
  i2c->address(address);
  if ((uint8_t) i2c->read(message.data8, 4) != 4)
  {
    return CL_RX_ERROR;
  }
  if (message.cl.instruction == CL_ERROR) // TODO change to something more descriptive
  {
    return CL_ERROR;
  }
  if (computeChecksum(message) != message.cl.checksum)
  {
    return CL_CHECKSUM_ERROR;
  }
  *position = message.cl.data;
  return CL_OK;
}

uint8_t Dynamixel::getPosition(uint16_t *position)
{
  uint8_t status;
  if (getPositionTx() != CL_OK)
  {
    errors++;
    std::cout << "TX Error!" << std::endl;
    resetI2c();
    status = CL_TX_ERROR;
  }
  status = getPositionRx(position);
  if (status != CL_OK)
  {
    resetI2c();
    if (status == CL_RX_ERROR)
    {
      std::cout << "RX error!" << std::endl;
    }
    if (status == CL_CHECKSUM_ERROR)
    {
      std::cout << "Checksum error!" << std::endl;
    }
  }
  return status;
}

uint8_t Dynamixel::getTestMessage(CLMessage32 *test_msg)
{
  CLMessage32 message;

  message.cl.instruction = 0x01;
  message.cl.data = 0;
  message.cl.checksum = 0;
  for (int i = 0; i < 3; i++)
  {
    message.cl.checksum += message.data8[i];
  }
  i2c->address(address);
  if (i2c->write(message.data8, 4) != MRAA_SUCCESS)
  {
    return CL_TX_ERROR;
  }

  i2c->address(address);
  if ((uint8_t) i2c->read(message.data8, 4) != 4)
  {
    return CL_RX_ERROR;
  }
  *test_msg = message;
  return CL_OK;
}

// TODO change naming to reflect velocity
// TODO align velocity with values found in dynamixel manual
// TODO add check on embedded system for runaway
uint16_t  Dynamixel::adjustPosition(int16_t adjustment)
{
  mutex.lock();
  uint16_t current_position;
  uint8_t status = getPosition(&current_position);
  if (status != CL_OK) // TODO do something more robust
  {
    std::cout << "Get Position Failed" << std::endl;
    mutex.unlock();
    return status;
  }
  int16_t next_position = 512 + adjustment;
  if (next_position > 1023)
  {
    setPosition(1023);
  }
  else if (next_position < 0)
  {
    setPosition(0);
  }
  else
  {
    setPosition((uint16_t)next_position);
  }

  // Update transform
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, ((double)current_position - 512.0) * (M_PI * 150.0) / (180.0 * 512.0));
  transform.setRotation(q);
  stamp = ros::Time::now();
  seq++;

  mutex.unlock();
  return current_position;
}

tf2::Transform Dynamixel::getTransform()
{
  return transform;
}

geometry_msgs::TransformStamped Dynamixel::getTransformMsg()
{
  geometry_msgs::TransformStamped transform_msg;
  transform_msg.transform = tf2::toMsg(transform);
  transform_msg.header.frame_id = frame_id;
  transform_msg.child_frame_id = child_frame_id;
  transform_msg.header.seq = seq;
  transform_msg.header.stamp = stamp;
  return transform_msg;
}

}

