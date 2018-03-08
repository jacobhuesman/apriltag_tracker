cmake_minimum_required(VERSION 3.5)
project(apriltag_tracker)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_generation
    nodelet
    std_msgs
    cmake_modules
    cv_bridge
    image_transport
    camera_info_manager
    dynamic_reconfigure
)

add_message_files(
    FILES
    AprilTagDetection.msg
    AprilTagDetectionArray.msg
    ATTLocalTiming.msg
    ATTGlobalTiming.msg
    ATTDiagnostics.msg
)

generate_messages(DEPENDENCIES
    std_msgs
    geometry_msgs
)


generate_dynamic_reconfigure_options(
    cfg/TFTest.cfg
    cfg/DynamicCamera.cfg
    cfg/DynamicServo.cfg
    cfg/DynamicAprilTagTracker.cfg
)

catkin_package(
    #INCLUDE_DIRS include
    #LIBRARIES apriltag_detector
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
    message_runtime nodelet std_msgs cv_bridge image_transport
)

set(raspicam_DIR /usr/local/lib/cmake)
find_package(raspicam REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.4.0 REQUIRED) # TODO check to make sure that compiler optimizations were used

include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

link_directories(/usr/src/gmock/build)

#############
# Libraries #
#############
add_library(Timers src/timers.cpp)
add_dependencies(Timers ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(CameraInfo src/camera_info.cpp)
target_link_libraries(CameraInfo ${OpenCV_LIBS})

add_library(Camera src/camera.cpp)
add_dependencies(Camera ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(Camera ${PROJECT_NAME}_gencfg)
target_link_libraries(Camera raspicam ${raspicam_LIBS})
target_link_libraries(Camera ${catkin_LIBRARIES})
target_link_libraries(Camera ${OpenCV_LIBS})
target_link_libraries(Camera CameraInfo)

add_library(Transform src/transform.cpp)
target_link_libraries(Transform ${catkin_LIBRARIES})

add_library(Tag src/tag.cpp)
target_link_libraries(Tag ${catkin_LIBRARIES})
target_link_libraries(Tag Transform)

add_library(TransformsCache src/transforms_cache.cpp)
target_link_libraries(TransformsCache ${catkin_LIBRARIES})
target_link_libraries(TransformsCache Tag)

add_library(AprilTagTracker src/apriltag_tracker.cpp)
add_dependencies(AprilTagTracker ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(Camera ${PROJECT_NAME}_gencfg)
target_link_libraries(AprilTagTracker raspicam ${raspicam_LIBS})
target_link_libraries(AprilTagTracker ${Eigen3_LIBS})
target_link_libraries(AprilTagTracker ${catkin_LIBRARIES})
target_link_libraries(AprilTagTracker ${OpenCV_LIBS})
target_link_libraries(AprilTagTracker apriltags)
target_link_libraries(AprilTagTracker CameraInfo)
target_link_libraries(AprilTagTracker Tag)

add_library(Dynamixel src/dynamixel.cpp)
add_dependencies(Dynamixel ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(Dynamixel ${PROJECT_NAME}_gencfg)
target_link_libraries(Dynamixel mraa)
target_link_libraries(Dynamixel ${catkin_LIBRARIES})

############
# Programs #
############
add_executable(apriltag_tracker src/apriltag_tracker_node.cpp)
add_dependencies(apriltag_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(apriltag_tracker ${catkin_LIBRARIES})
target_link_libraries(apriltag_tracker ${OpenCV_LIBS})
target_link_libraries(apriltag_tracker AprilTagTracker)
target_link_libraries(apriltag_tracker Camera)
target_link_libraries(apriltag_tracker Dynamixel)
target_link_libraries(apriltag_tracker Timers)
target_link_libraries(apriltag_tracker TransformsCache)

add_executable(comm_tool src/util/comm_tool.cpp)
target_link_libraries(comm_tool Dynamixel)

add_executable(speed_test src/util/speed_test.cpp)

#add_executable(tf_test src/util/tf_test.cpp)
#add_dependencies(tf_test ${PROJECT_NAME}_gencfg)
#target_link_libraries(tf_test ${catkin_LIBRARIES})

add_executable(transform_cache_test src/util/transform_cache_test.cpp)
target_link_libraries(transform_cache_test ${catkin_LIBRARIES})

add_executable(transforms_publisher src/util/transforms_publisher.cpp)
target_link_libraries(transforms_publisher ${catkin_LIBRARIES})

#########
# Tests #
#########
catkin_add_gtest(servo_tests src/tests/servo_tests.cpp)
if(TARGET servo_tests)
  target_link_libraries(servo_tests ${catkin_LIBRARIES})
  target_link_libraries(servo_tests Dynamixel)
  target_link_libraries(servo_tests Transform)
  target_link_libraries(servo_tests gmock_main)
endif()

catkin_add_gtest(tag_tests src/tests/tag_tests.cpp)
if(TARGET tag_tests)
  target_link_libraries(tag_tests ${catkin_LIBRARIES})
  target_link_libraries(tag_tests Tag)
endif()

catkin_add_gtest(transform_tests src/tests/transform_tests.cpp)
if(TARGET transform_tests)
  target_link_libraries(transform_tests ${catkin_LIBRARIES})
  target_link_libraries(transform_tests Transform)
endif()

catkin_add_gtest(timer_tests src/tests/timer_tests.cpp)
if(TARGET timer_tests)
  target_link_libraries(timer_tests Timers)
endif()

catkin_add_gtest(camera_tests src/tests/camera_tests.cpp)
if(TARGET camera_tests)
  target_link_libraries(camera_tests Camera)
endif()

catkin_add_gtest(apriltag_tracker_tests src/tests/apriltag_tracker_tests.cpp)
if(TARGET apriltag_tracker_tests)
  target_link_libraries(apriltag_tracker_tests AprilTagTracker)
  target_link_libraries(apriltag_tracker_tests TransformsCache)
endif()

