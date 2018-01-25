#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <boost/program_options.hpp>

#include <host_comm_layer.h>
#include <comm_layer_defs.h>

using namespace HostCommLayer;

void printStatus(uint8_t status)
{
  if (status == CL_OK)
  {
    std::cout << "Success" << std::endl;
  }
  else if (status == CL_TX_ERROR)
  {
    std::cout << "TX Error" << std::endl;
  }
  else if (status == CL_RX_ERROR) {
    std::cout << "RX Error" << std::endl;
  }
  else
  {
    std::cout << "System returned: " << status << std::endl;
  }

}

using namespace boost::program_options;

int main(int argc, char *argv[])
{
  Dynamixel servo(0x11);

  try
  {
    options_description desc{"Options"};
    desc.add_options()
        ("help,h", "Help screen")
        ("setPosition,s",  value<uint16_t >()->default_value(512), "Set Position")
        ("getPosition,g", "Get Position")
        ("setVelocity,v",  value<uint16_t >()->default_value(512), "Set Velocity");
    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);

    if (vm.count("help"))
    {
      std::cout << desc << '\n';
    }
    else if (vm.count("setPosition"))
    {
      uint16_t position = vm["setPosition"].as<uint16_t>();
      std::cout << "Setting position: " << position << std::endl;
      uint8_t status = servo.setPosition(position);
      printStatus(status);
    }
    else if (vm.count("getPosition"))
    {
      uint16_t position;
      uint8_t status = servo.getPosition(&position);
      if (status == CL_OK)
      {
        std::cout << "Read position: " << position << std::endl;
      }
      else
      {
        printStatus(status);
      }
    }
    else if (vm.count("setVelocity"))
    {
      uint16_t velocity = vm["setVelocity"].as<uint16_t>();
      uint8_t status = servo.setVelocity(velocity);
      if (status == CL_OK)
      {
        std::cout << "Setting velocity: " << velocity << std::endl;
      }
      else
      {
        printStatus(status);
      }
    }
  }
  catch (const error &ex)
  {
    std::cerr << ex.what() << '\n';
  }
  /*int c;
  while ((c = getopt(argc, argv, "v:s:ght")) != -1)
  {
    switch (c)
    {
      case 's':
      {
        position = (uint16_t)std::stoi(optarg);
        std::cout << "Setting position: " << position << std::endl;
        uint8_t status = servo.setPosition(position);
        printStatus(status);
        break;
      }
      case 'g':
      {
        uint8_t status = servo.getPosition(&position);
        if (status == CL_OK)
        {
          std::cout << "Read position: " << position << std::endl;
        }
        else
        {
          printStatus(status);
        }
        break;
      }
      case 'v':
      {
        uint16_t velocity = (uint16_t)std::stoi(optarg);
        uint8_t status = servo.setVelocity(velocity);
        printStatus(status);
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
  }*/
}
