cmake_minimum_required(VERSION 3.5)
project(apriltag_tracker)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    image_transport
    roscpp
    sensor_msgs
    geometry_msgs
    tf
    cv_bridge
    message_generation
    nodelet
    std_msgs
    cmake_modules
)

add_message_files(
    FILES
    AprilTagDetection.msg
    AprilTagDetectionArray.msg
)

generate_messages(DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    #LIBRARIES apriltag_detector
    CATKIN_DEPENDS image_transport roscpp sensor_msgs geometry_msgs tf cv_bridge message_runtime nodelet std_msgs
    DEPENDS Eigen3
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package (Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(Eigen3)

add_executable(AprilTag_Tracker src/main.cpp)
target_link_libraries(AprilTag_Tracker ${OpenCV_LIBS})
target_link_libraries(AprilTag_Tracker ${Eigen3_LIBS})
target_link_libraries(AprilTag_Tracker apriltags)