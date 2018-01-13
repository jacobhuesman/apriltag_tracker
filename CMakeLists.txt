cmake_minimum_required(VERSION 3.5)
project(apriltag_tracker)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    tf
    message_generation
    nodelet
    std_msgs
    cmake_modules
    cv_bridge
    image_transport
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
    #INCLUDE_DIRS include
    #LIBRARIES apriltag_detector
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf message_runtime nodelet std_msgs cv_bridge image_transport
)

#TODO ROS by default uses OpenCV 3, should check to see if it's worth switching over
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

set(raspicam_DIR /usr/local/lib/cmake)
find_package(raspicam REQUIRED)
include_directories()

add_library(HostCommLayer src/host_comm_layer.cpp)
target_link_libraries(HostCommLayer mraa)

add_executable(apriltag_tracker src/main.cpp)
add_dependencies(apriltag_tracker apriltag_tracker_generate_messages_cpp)
target_link_libraries(apriltag_tracker ${Eigen3_LIBS})
target_link_libraries(apriltag_tracker apriltags)
target_link_libraries(apriltag_tracker raspicam ${raspicam_LIBS})
target_link_libraries(apriltag_tracker ${catkin_LIBRARIES})
target_link_libraries(apriltag_tracker ${OpenCV_LIBS})

add_executable(test_i2c src/test.cpp)
target_link_libraries(test_i2c HostCommLayer)
