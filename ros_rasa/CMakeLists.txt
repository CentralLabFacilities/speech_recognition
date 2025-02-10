cmake_minimum_required(VERSION 3.5)
project(ros_rasa)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/rasa_nlu_node
  scripts/rasa_nlu_node_asr
  scripts/train_model.py
  scripts/test_nlu_model.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/test_nlu.py
  TYPE BIN
)

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
