cmake_minimum_required(VERSION 3.5)
project(apriltag_tracker)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    # standard
    roscpp
    dynamic_reconfigure
    # custom
    dynamixel
    # messages
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2_geometry_msgs
    message_generation
    # tf
    tf2
    tf2_ros
    # images
    cv_bridge
    image_transport
    compressed_image_transport
    theora_image_transport
    camera_info_manager
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
    cfg/DynamicAprilTagTracker.cfg
)

catkin_package(
    #INCLUDE_DIRS include
    #LIBRARIES apriltag_detector
    CATKIN_DEPENDS
        # standard
        roscpp
        dynamic_reconfigure
        # custom
        dynamixel
        # messages
        std_msgs
        sensor_msgs
        geometry_msgs
        tf2_geometry_msgs
        message_generation
        # tf
        tf2
        tf2_ros
        # images
        cv_bridge
        image_transport
        compressed_image_transport
        theora_image_transport
        camera_info_manager
)

set(raspicam_DIR /usr/local/lib/cmake)
find_package(raspicam)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3.4.0 REQUIRED) # TODO check to make sure that compiler optimizations were used

include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

# Add gmock
link_directories(/usr/src/gmock/build)
set(GTEST_LIBRARIES gmock_main)

#############
# Libraries #
#############
add_library(Timers src/timers.cpp)
add_dependencies(Timers ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(CameraInfo src/camera_info.cpp)
target_link_libraries(CameraInfo ${OpenCV_LIBS})

if (raspicam_FOUND)
  add_library(Camera src/camera.cpp)
  add_dependencies(Camera ${${PROJECT_NAME}_EXPORTED_TARGETS})
  add_dependencies(Camera ${PROJECT_NAME}_gencfg)
  target_link_libraries(Camera raspicam ${raspicam_LIBS})
  target_link_libraries(Camera ${catkin_LIBRARIES})
  target_link_libraries(Camera ${OpenCV_LIBS})
  target_link_libraries(Camera CameraInfo)
else()
  message("[WARNING] raspicam not found")
endif()

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
add_dependencies(AprilTagTracker ${PROJECT_NAME}_gencfg)
target_link_libraries(AprilTagTracker raspicam ${raspicam_LIBS})
target_link_libraries(AprilTagTracker ${Eigen3_LIBS})
target_link_libraries(AprilTagTracker ${catkin_LIBRARIES})
target_link_libraries(AprilTagTracker ${OpenCV_LIBS})
target_link_libraries(AprilTagTracker apriltags)
target_link_libraries(AprilTagTracker CameraInfo)
target_link_libraries(AprilTagTracker Tag)

############
# Programs #
############
if (raspicam_FOUND)
  add_executable(apriltag_tracker src/apriltag_tracker_node.cpp)
  add_dependencies(apriltag_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS})
  target_link_libraries(apriltag_tracker ${catkin_LIBRARIES})
  target_link_libraries(apriltag_tracker ${OpenCV_LIBS})
  target_link_libraries(apriltag_tracker AprilTagTracker)
  target_link_libraries(apriltag_tracker Camera)
  target_link_libraries(apriltag_tracker Dynamixel)
  target_link_libraries(apriltag_tracker Timers)
  target_link_libraries(apriltag_tracker TransformsCache)
endif()

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

if (raspicam_FOUND)
  catkin_add_gtest(camera_tests src/tests/camera_tests.cpp)
  if(TARGET camera_tests)
    target_link_libraries(camera_tests Camera)
  endif()
endif()

catkin_add_gtest(apriltag_tracker_tests src/tests/apriltag_tracker_tests.cpp)
if(TARGET apriltag_tracker_tests)
  target_link_libraries(apriltag_tracker_tests AprilTagTracker)
  target_link_libraries(apriltag_tracker_tests TransformsCache)
endif()

