cmake_minimum_required(VERSION 3.7)
project(AprilTag_Tracker)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(Eigen3)

add_executable(AprilTag_Tracker src/main.cpp)
target_link_libraries(AprilTag_Tracker ${OpenCV_LIBS})
target_link_libraries(AprilTag_Tracker Eigen3::Eigen)
target_link_libraries(AprilTag_Tracker apriltags)