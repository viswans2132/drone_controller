cmake_minimum_required(VERSION 2.8.3)
project(drone_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES  pid_controller
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs 
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(pid_controller
  src/libs/pid_controller.cpp
)

add_library(asm_controller
  src/libs/asm_controller.cpp
)

target_link_libraries(pid_controller ${catkin_LIBRARIES})
add_dependencies(pid_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(asm_controller ${catkin_LIBRARIES})
add_dependencies(asm_controller ${catkin_EXPORTED_TARGETS})


add_executable(pid_node src/nodes/pid_node.cpp)
add_dependencies(pid_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pid_node
  pid_controller ${catkin_LIBRARIES})

add_executable(asm_node src/nodes/asm_node.cpp)
add_dependencies(asm_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(asm_node
  asm_controller ${catkin_LIBRARIES})

install(TARGETS pid_controller asm_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS pid_node asm_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


