#include <unistd.h>
#include <signal.h>
#include <iostream>

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

uint8_t rx_buf[MAX_BUFFER_LENGTH];
uint8_t tx_buf[MAX_BUFFER_LENGTH];
int main(int argc, char *argv[])
{
  mraa::I2c i2c(0);

  i2c.address(I2C_ADDR);
  tx_buf[0] = 0x40;
  tx_buf[1] = 0x00;
  tx_buf[2] = (char)argc;
  tx_buf[3] = 0x25;
  i2c.write(tx_buf, 4); 

  i2c.address(I2C_ADDR);
  int worked = i2c.read(rx_buf, 4);

  std::cout << "Read: " << worked << std::endl;
  std::cout << (int)rx_buf[0] << ":" << (int)rx_buf[1] << ":" << (int)rx_buf[2] << ":" << (int)rx_buf[3] <<  std::endl;
}
