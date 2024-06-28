cmake_minimum_required(VERSION 3.0.2)
project(takeoff_landing_sys)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  cv_bridge
  sensor_msgs
  ttauav_node
  geometry_msgs
  # message_generation
)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES takeoff_landing_sys
  CATKIN_DEPENDS roscpp std_msgs ttauav_node geometry_msgs
  DEPENDS OpenCV
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/takeoff_landing_sys.cpp
# )
add_executable(${PROJECT_NAME}_node src/takeoff_landing_sys.cpp)
## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)


target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

