cmake_minimum_required(VERSION 3.5)
project(ros_piper_tts)

find_package(catkin REQUIRED COMPONENTS
  rospy    
  dynamic_reconfigure
  )

catkin_python_setup()

catkin_package()

install(PROGRAMS
 scripts/piper_node
 scripts/piper_node_palforward
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

