cmake_minimum_required(VERSION 3.0.2)
project(clf_speech_rqt)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED)

catkin_python_setup()

# Declare a catkin package
catkin_package()


#############
## Install ##
#############

# Install resources
install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install python scripts
catkin_install_python(PROGRAMS scripts/rqt_clf_speech
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install plugins
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
