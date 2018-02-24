#include <dynamixel_host_layer.h>
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
  frame_id = "servo_base_link"; // TODO parametrize
  child_frame_id = "servo_joint"; // TODO parametrize

  max_velocity = 50; // TODO parametrize
  desired_velocity = 0;
  current_velocity = 0;
  last_velocity_update = (ros::Time::now() - ros::Duration(1.0));
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
  return ~checksum;
}

//TODO add error checking for both setPosition and getPosition
uint8_t Dynamixel::setPosition(uint16_t position)
{
  CLMessage32 message;
  i2c->address(address);
  message.ucl.instruction = DYN_SET_POSITION;
  message.ucl.data = position;
  message.ucl.checksum = computeChecksum(message);
  if (i2c->write(message.data8, 4) != MRAA_SUCCESS)
  {
    return CL_TX_ERROR;
  }
  if (i2c->read(message.data8, 4) != 4)
  {
    return CL_RX_ERROR;
  }
  return CL_OK;
}

uint8_t Dynamixel::setVelocity(uint16_t velocity)
{
  CLMessage32 message;
  i2c->address(address);
  message.ucl.instruction = DYN_SET_VELOCITY;
  message.ucl.data = velocity;
  message.ucl.checksum = computeChecksum(message);
  if (i2c->write(message.data8, 4) != MRAA_SUCCESS)
  {
    return CL_TX_ERROR;
  }
  if (i2c->read(message.data8, 4) != 4)
  {
    return CL_RX_ERROR;
  }
  return message.ucl.instruction;
}

uint8_t Dynamixel::updateDesiredVelocity(int16_t velocity)
{
  mutex.lock();
  desired_velocity = velocity;
  last_velocity_update = ros::Time::now();
  mutex.unlock();
}

uint8_t Dynamixel::setPollingDt(uint16_t polling_dt)
{
  CLMessage32 message;
  i2c->address(address);
  message.ucl.instruction = DYN_SET_POLLING_DT;
  message.ucl.data = polling_dt;
  message.ucl.checksum = computeChecksum(message);
  if (i2c->write(message.data8, 4) != MRAA_SUCCESS)
  {
    return CL_TX_ERROR;
  }
  if (i2c->read(message.data8, 4) != 4)
  {
    return CL_RX_ERROR;
  }
  return message.ucl.instruction;
}


  uint8_t Dynamixel::getPositionTx()
{
  CLMessage32 message;
  i2c->address(address);
  message.ucl.instruction = DYN_GET_POSITION_INSTRUCTION;
  message.ucl.data = 0;
  message.ucl.checksum = computeChecksum(message);
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
  if (message.ucl.instruction == CL_ERROR) // TODO change to something more descriptive
  {
    return CL_ERROR;
  }
  if (computeChecksum(message) != message.ucl.checksum)
  {
    return CL_CHECKSUM_ERROR;
  }
  *position = message.ucl.data;
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

  message.ucl.instruction = 0x01;
  message.ucl.data = 0;
  message.ucl.checksum = 0;
  for (int i = 0; i < 3; i++)
  {
    message.ucl.checksum += message.data8[i];
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

uint8_t Dynamixel::adjustCamera(int16_t velocity)
{
  mutex.lock();

  // Read position
  uint16_t current_position;
  uint8_t status = getPosition(&current_position);
  if (status != CL_OK) // TODO do something more robust
  {
    std::cout << "Get Position Failed" << std::endl;
    mutex.unlock();
    return status;
  }

  // Throttle velocity
  if (abs(velocity) > max_velocity)
  {
    if (velocity < 0)
    {
      velocity = -max_velocity;
    }
    else
    {
      velocity = max_velocity;
    }
  }

  // Send control message
  CLMessage32 message;
  i2c->address(address);
  message.cl.instruction = DYN_ADJUST_SERVO;
  message.cl.data = velocity;
  message.cl.checksum = computeChecksum(message);
  if (i2c->write(message.data8, 4) != MRAA_SUCCESS)
  {
    mutex.unlock();
    return CL_TX_ERROR;
  }
  if (i2c->read(message.data8, 4) != 4)
  {
    mutex.unlock();
    return CL_RX_ERROR;
  }
  current_velocity = velocity;

  // Update transform
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, ((double)current_position - 512.0) * (M_PI * 150.0) / (180.0 * 512.0)); // TODO make sure this is correct
  transform.setRotation(q);
  stamp = ros::Time::now();
  seq++;

  mutex.unlock();
  return CL_OK;
}

uint8_t Dynamixel::scan()
{
  mutex.lock();

  // Read position
  uint16_t current_position;
  uint8_t status = getPosition(&current_position);
  if (status != CL_OK) // TODO do something more robust
  {
    std::cout << "Get Position Failed" << std::endl;
    mutex.unlock();
    return status;
  }

  // Send control message
  CLMessage32 message;
  i2c->address(address);
  message.cl.instruction = DYN_ADJUST_SERVO;

  // If we aren't moving pick the direction that covers the most area
  if (current_velocity == 0)
  {
    if (current_position <= 512)
    {
      message.cl.data = max_velocity;
    }
    else
    {
      message.cl.data = -max_velocity;
    }
  }

  // If we are moving continue moving in the same direction but faster
  else if (current_velocity > 0)
  {
    message.cl.data = max_velocity;
  }
  else if (current_velocity < 0)
  {
    message.cl.data = -max_velocity;
  }

  // If we've hit the end, change direction
  if (current_position >= 1011)
  {
    message.cl.data = -max_velocity;
  }
  if (current_position <= 12)
  {
    message.cl.data = max_velocity;
  }
  current_velocity = message.cl.data;

  // Send decision
  message.cl.checksum = computeChecksum(message);
  if (i2c->write(message.data8, 4) != MRAA_SUCCESS)
  {
    mutex.unlock();
    std::cout << "TX Error" << std::endl;
    return CL_TX_ERROR;
  }
  if (i2c->read(message.data8, 4) != 4)
  {
    mutex.unlock();
    std::cout << "RX Error" << std::endl;
    return CL_RX_ERROR;
  }

  // Update transform
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, ((double)current_position - 512.0) * (M_PI * 150.0) / (180.0 * 512.0)); // TODO make sure this is correct
  transform.setRotation(q);
  stamp = ros::Time::now();
  seq++;

  mutex.unlock();
  return CL_OK;
}

tf2::Transform Dynamixel::getTransform()
{
  return this->transform;
}

tf2::Stamped<tf2::Transform> Dynamixel::getStampedTransform()
{
  tf2::Stamped<tf2::Transform> transform(this->transform, stamp, frame_id); // TODO probably should just do this right away
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

ros::Time Dynamixel::getLastVelocityUpdate()
{
  return last_velocity_update;
}

int16_t Dynamixel::getDesiredVelocity()
{
  return desired_velocity;
}

}

