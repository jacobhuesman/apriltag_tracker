#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <string>
#include <getopt.h>
#include <cstdint>

#include <comm_layer_defs.h>

#include "mraa.hpp"
#include "math.h"

#define MAX_BUFFER_LENGTH 6
#define I2C_ADDR 0x11

int running = 0;

void sig_handler(int signo)
{
  if (signo == SIGINT) {
      printf("closing nicely\n");
      running = -1;
  }
}

//TODO add error checking for both setPosition and getPosition
void setPosition(uint16_t position)
{
  CLMessage32 message;

  mraa::I2c i2c(0);

  i2c.address(I2C_ADDR);
  message.cl.instruction = DYN_SET_POSITION;
  message.cl.data = position;
  message.cl.checksum = 0;
  for (int i = 0; i < 3; i++)
  {
    message.cl.checksum += message.data8[i];
  }
  int status = i2c.write(message.data8, 4);
}

void getPosition(uint16_t *position)
{
  CLMessage32 message;

  mraa::I2c i2c(0);

  i2c.address(I2C_ADDR);
  message.cl.instruction = DYN_GET_POSITION_INSTRUCTION;
  message.cl.data = 0;
  message.cl.checksum = 0;
  for (int i = 0; i < 3; i++)
  {
    message.cl.checksum += message.data8[i];
  }
  int status = i2c.write(message.data8, 4);

  i2c.address(I2C_ADDR);
  int bytes_read = i2c.read(message.data8, 4);
  *position = message.cl.data;
}


int main(int argc, char *argv[])
{
  int c;
  uint16_t position;
  while ((c = getopt(argc, argv, "s:gh")) != -1)
  {
    switch (c)
    {
      case 's':
      {
        position = (uint16_t)std::stoi(optarg);
        std::cout << "Setting position: " << position << std::endl;
        setPosition(position);
        break;
      }
      case 'g':
      {
        getPosition(&position);
        std::cout << "Read position: " << position << std::endl;
        break;
      }
      case 'h':
      {
        std::cout << "Options: " << std::endl;
        std::cout << " -s <pos> : set position" << std::endl;
        std::cout << " -g       : get position" << std::endl;
        std::cout << " -h       : get help"     << std::endl;
        break;
      }
      case '?':
      {
        std::cout << "Invalid option" << std::endl;
        abort();
      }
      default:
      {
        std::cout << "Please select an option" << std::endl;
        abort();
      }
    }
  }


}
