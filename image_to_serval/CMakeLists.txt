cmake_minimum_required(VERSION 2.8.3)
project(image_to_serval)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  topic_tools
  nodelet
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS  ${catkin_INCLUDE_DIRS}
#  LIBRARIES task_msgs
   CATKIN_DEPENDS   cv_bridge image_transport roscpp sensor_msgs topic_tools nodelet
#  DEPENDS system_lib
)

add_library(image_to_serval src/image_to_serval.cpp)
target_link_libraries(image_to_serval ${catkin_LIBRARIES})

add_executable(image_to_serval_node src/image_to_serval_node.cpp)
target_link_libraries(image_to_serval_node image_to_serval)

add_library(image_to_serval_nodelet src/image_to_serval_nodelet.cpp)
target_link_libraries(image_to_serval_nodelet image_to_serval)



