#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <chrono>



int main(int argc, char *argv[])
{
  // Timing info
  std::chrono::system_clock::time_point start, finish;
  std::chrono::milliseconds calc_time;

  // Start
  long cycles = 100000000;
  std::cout << "Number of cycles: " << cycles << std::endl << std::endl;

  // Single-precision floating point
  float float_test;

  std::cout << "Float addition: ";
  float_test = 0.0f;
  start = std::chrono::system_clock::now();
  for (long i = 0; i < cycles; i++)
  {
    float_test = float_test + 2;
  }
  finish = std::chrono::system_clock::now();
  calc_time = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start);
  std::cout << calc_time.count() << " ms" << std::endl;

  std::cout << "Float multiplication: ";
  float_test = 0.0f;
  start = std::chrono::system_clock::now();
  for (long i = 0; i < cycles; i++)
  {
    float_test = float_test * 2;
  }
  finish = std::chrono::system_clock::now();
  calc_time = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start);
  std::cout << calc_time.count() << " ms" << std::endl;

  // Double-precision floating point
  double double_test;

  std::cout << "Double addition: ";
  double_test = 0.0;
  start = std::chrono::system_clock::now();
  for (long i = 0; i < cycles; i++)
  {
    double_test = double_test + 2;
  }
  finish = std::chrono::system_clock::now();
  calc_time = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start);
  std::cout << calc_time.count() << " ms" << std::endl;

  std::cout << "Double multiplication: ";
  double_test = 0.0;
  start = std::chrono::system_clock::now();
  for (long i = 0; i < cycles; i++)
  {
    double_test = double_test * 2;
  }
  finish = std::chrono::system_clock::now();
  calc_time = std::chrono::duration_cast<std::chrono::milliseconds>(finish - start);
  std::cout << calc_time.count() << " ms" << std::endl;
}