cmake_minimum_required(VERSION 3.5)
project(ros_nllb)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/translating_node
  scripts/ct2_node

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)