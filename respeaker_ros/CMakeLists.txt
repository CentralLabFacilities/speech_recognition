cmake_minimum_required(VERSION 2.8.3)
project(respeaker_ros)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/RespeakerArray2.cfg)

catkin_package()

catkin_install_python(
  PROGRAMS
  scripts/respeaker_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
