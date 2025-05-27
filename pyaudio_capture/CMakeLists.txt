cmake_minimum_required(VERSION 3.16)
project(pyaudio_capture)

find_package(catkin REQUIRED COMPONENTS
  rospy
  audio_common_msgs
  
)
include_directories(${catkin_INCLUDE_DIRS})

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/recorder_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)