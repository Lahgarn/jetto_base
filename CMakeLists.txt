cmake_minimum_required(VERSION 2.8.3)
project(jetto_base)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  # core
  roscpp
  rospy
  roslaunch
  # messages
  std_msgs
  sensor_msgs
  message_generation
  # images
  cv_bridge
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(OpenCV 4.1 REQUIRED)

# Python
catkin_python_setup()

# Add launch files
roslaunch_add_file_check(launch)

##############
## Messages ##
##############

add_message_files(
  FILES
  RawMotor.msg
  BatteryStatus.msg
  Point2D.msg
  Keypoints.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES jetto_base
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
   CATKIN_DEPENDS rospy
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

# CPP

include_directories(
  src/cpp
  ${catkin_INCLUDE_DIRS}
)

add_executable(camera
    src/cpp/camera/camera_node.cpp
    src/cpp/camera/camera.cpp
)

target_link_libraries(camera ${catkin_LIBRARIES} ${OpenCV_LIBS})


add_executable(optic_flow
    src/cpp/optic_flow/of_node.cpp
    src/cpp/optic_flow/of_calculator.cpp
)

target_link_libraries(optic_flow ${catkin_LIBRARIES} ${OpenCV_LIBS})

#############
## Install ##
#############

install(PROGRAMS
     nodes/motor.py
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
