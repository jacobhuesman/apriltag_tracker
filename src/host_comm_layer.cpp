#include <host_comm_layer.h>
#include <comm_layer_defs.h>
#include <iostream>

namespace HostCommLayer
{

Dynamixel::Dynamixel(uint8_t i2c_address)
{
  i2c = new mraa::I2c(0);
  this->address = i2c_address;
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
  if (computeChecksum(message) != message.cl.checksum)
  {
    return CL_CHECKSUM_ERROR;
  }
  *position = message.cl.data;
  return CL_OK;
}

uint8_t Dynamixel::getPosition(uint16_t *position)
{
  if (getPositionTx() != CL_OK)
  {
    errors++;
    std::cout << "TX Error!" << std::endl;
    return CL_TX_ERROR;
  }
  uint8_t status = getPositionRx(position);
  if (status != CL_OK)
  {
    if (status == CL_RX_ERROR)
    {
      std::cout << "RX error!" << std::endl;
    }
    if (status == CL_CHECKSUM_ERROR)
    {
      std::cout << "Checksum error!" << std::endl;
    }
    return status;
  }
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

}

