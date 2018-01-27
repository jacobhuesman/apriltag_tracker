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
find_package(OpenCV 3 REQUIRED)

include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)

add_library(Camera src/camera.cpp)
add_dependencies(Camera ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(Camera ${PROJECT_NAME}_gencfg)
target_link_libraries(Camera raspicam ${raspicam_LIBS})
target_link_libraries(Camera ${catkin_LIBRARIES})
target_link_libraries(Camera ${OpenCV_LIBS})

add_library(AprilTagTracker src/apriltag_tracker.cpp)
add_dependencies(AprilTagTracker ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(AprilTagTracker raspicam ${raspicam_LIBS})
target_link_libraries(AprilTagTracker ${Eigen3_LIBS})
target_link_libraries(AprilTagTracker ${catkin_LIBRARIES})
target_link_libraries(AprilTagTracker ${OpenCV_LIBS})
target_link_libraries(AprilTagTracker apriltags)
target_link_libraries(AprilTagTracker HostCommLayer)
target_link_libraries(AprilTagTracker Camera)

add_library(HostCommLayer src/host_comm_layer.cpp)
target_link_libraries(HostCommLayer mraa)
target_link_libraries(HostCommLayer ${catkin_LIBRARIES})


add_executable(apriltag_tracker src/apriltag_tracker_node.cpp)
add_dependencies(apriltag_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(apriltag_tracker ${catkin_LIBRARIES})
target_link_libraries(apriltag_tracker ${OpenCV_LIBS})
target_link_libraries(apriltag_tracker AprilTagTracker)

add_executable(comm_test src/util/comm_test.cpp)
target_link_libraries(comm_test HostCommLayer)

add_executable(speed_test src/util/speed_test.cpp)

#add_executable(tf_test src/util/tf_test.cpp)
#add_dependencies(tf_test ${PROJECT_NAME}_gencfg)
#target_link_libraries(tf_test ${catkin_LIBRARIES})

add_executable(transform_cache_test src/util/transform_cache_test.cpp)
target_link_libraries(transform_cache_test ${catkin_LIBRARIES})

add_executable(static_transforms_publisher src/util/static_transforms_publisher.cpp)
target_link_libraries(static_transforms_publisher ${catkin_LIBRARIES})

