cmake_minimum_required(VERSION 3.0.2)
project(drone_control)


add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  rotors_control
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  LIBRARIES drone_control
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs rotors_control
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(drone_control
  src/library/drone_control.cpp
)

target_link_libraries(drone_control ${catkin_LIBRARIES})
add_dependencies(drone_control ${catkin_EXPORTED_TARGETS})

add_executable(drone_control_node src/nodes/drone_control_node.cpp)
add_dependencies(drone_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(drone_control_node
  drone_control ${catkin_LIBRARIES})

install(TARGETS drone_control
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS drone_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
