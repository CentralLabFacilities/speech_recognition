cmake_minimum_required(VERSION 2.8.3)
project(respeaker_ros)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/RespeakerArray2.cfg
  cfg/Respeaker3800.cfg
)

catkin_package()

catkin_install_python(
  PROGRAMS
  scripts/respeaker_node_array2
  scripts/respeaker_node_3800
  scripts/respeaker_node_3800_echo
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)
