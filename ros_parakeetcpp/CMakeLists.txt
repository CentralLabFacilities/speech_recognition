cmake_minimum_required(VERSION 3.16)
project(
  ros_parakeetcpp
  VERSION 0.1.0
  LANGUAGES CXX
)

#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

#add_compile_options(-fsanitize=undefined)
#add_link_options(-fsanitize=undefined)
set(CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake
    ${CMAKE_MODULE_PATH}
)
find_package(Parakeet REQUIRED)

##################################
## Deps
##################################
find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    clf_speech_msgs
    )


##################################
## CATKIN
##################################
catkin_package()

##############################
##  Build
##############################
## ECWM EXEC
add_executable(ros_parakeetcpp
    src/node.cpp
    src/parakeet.cpp
)
target_include_directories(ros_parakeetcpp PRIVATE include ${catkin_INCLUDE_DIRS} ${Parakeet_INCLUDE_DIR})
target_link_libraries(ros_parakeetcpp ${catkin_LIBRARIES} ${Parakeet_LIBRARY})


##############################
#catkin_python_setup()

##############################
## Install rules
##############################
install(TARGETS
        ros_parakeetcpp
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

