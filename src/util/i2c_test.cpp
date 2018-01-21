#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <chrono>

#include <host_comm_layer.h>
#include <comm_layer_defs.h>

using namespace HostCommLayer;

int main(int argc, char *argv[])
{
  /* I2C */
  Dynamixel servo(0x11);

  /* Other */
  uint16_t position;

  /* Start */
  int c;
  while ((c = getopt(argc, argv, "s:ght")) != -1)
  {
    switch (c)
    {
      case 's':
      {
        position = (uint16_t)std::stoi(optarg);
        std::cout << "Setting position: " << position << std::endl;
        servo.setPosition(position);
        break;
      }
      case 'g':
      {
        servo.getPosition(&position);
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
      case 't':
      {
        std::cout << "Running getPosition test" << std::endl;
        std::chrono::system_clock::time_point start, end;
        std::chrono::microseconds difference;
        long running_count = 0;
        CLMessage32 test_msg;
        long i = 0;
        for (i = 0; i < 100000; i++)
        {
          start = std::chrono::system_clock::now();
          position = servo.adjustPosition(0);
          end = std::chrono::system_clock::now();
          difference = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
          std::cout << "Test " << std::setw(3) << i << " time: " << std::setw(8) << difference.count();
          std::cout << "us, position: " << std::setw(4) << position << std::endl;
          running_count += difference.count();
          usleep(100);
        }
        std::cout << "Test results: " << running_count/i << "us" << std::endl;
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
