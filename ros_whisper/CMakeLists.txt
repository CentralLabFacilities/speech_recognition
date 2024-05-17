cmake_minimum_required(VERSION 2.8.12)
project(ros_whisper)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/testing_node
  scripts/live_whisper_node
  scripts/whisper_transcribe_node
  scripts/fast-whisper_transcribe_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
