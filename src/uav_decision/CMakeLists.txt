cmake_minimum_required(VERSION 3.0.2)
project(uav_decision)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  ttauav_node
  geometry_msgs
  image_transport
  cv_bridge
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uav_decision
  CATKIN_DEPENDS roscpp std_msgs ttauav_node geometry_msgs
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(uav_decision_node src/uav_decision.cpp src/main.cpp)

add_dependencies(uav_decision_node 
  ${uav_decision_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(uav_decision_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
