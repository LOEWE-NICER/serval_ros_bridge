cmake_minimum_required(VERSION 2.8.3)
project(serval_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  genmsg
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  NewServalFile.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serval_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########


#############
## Install ##
#############


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_serval_msgs.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
