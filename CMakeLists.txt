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
find_package(OsqpEigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES  pid_controller asm_controller rsb_controller ec_controller aec_controller ecbf_controller multi_ecbf_controller
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

add_library(rsb_controller
  src/libs/rsb_controller.cpp
)

add_library(ec_controller
  src/libs/ec_controller.cpp
)

add_library(aec_controller
  src/libs/aec_controller.cpp
)

add_library(ecbf_controller
  src/libs/ecbf_controller.cpp
)

add_library(multi_ecbf_controller
  src/libs/multi_ecbf_controller.cpp
)

target_link_libraries(pid_controller ${catkin_LIBRARIES})
add_dependencies(pid_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(asm_controller ${catkin_LIBRARIES})
add_dependencies(asm_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(rsb_controller ${catkin_LIBRARIES})
add_dependencies(rsb_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(ec_controller ${catkin_LIBRARIES})
add_dependencies(ec_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(aec_controller ${catkin_LIBRARIES})
add_dependencies(aec_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(ecbf_controller ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)
add_dependencies(ecbf_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(multi_ecbf_controller ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)
add_dependencies(multi_ecbf_controller ${catkin_EXPORTED_TARGETS})


add_executable(pid_node src/nodes/pid_node.cpp)
add_dependencies(pid_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(pid_node
  pid_controller ${catkin_LIBRARIES})

add_executable(asm_node src/nodes/asm_node.cpp)
add_dependencies(asm_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(asm_node
  asm_controller ${catkin_LIBRARIES})

add_executable(rsb_node src/nodes/rsb_node.cpp)
add_dependencies(rsb_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(rsb_node
  rsb_controller ${catkin_LIBRARIES})

add_executable(ec_node src/nodes/ec_node.cpp)
add_dependencies(ec_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ec_node
  ec_controller ${catkin_LIBRARIES})

add_executable(aec_node src/nodes/aec_node.cpp)
add_dependencies(aec_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(aec_node
  aec_controller ${catkin_LIBRARIES})

add_executable(ecbf_node src/nodes/ecbf_node.cpp)
add_dependencies(ecbf_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ecbf_node
  ecbf_controller ${catkin_LIBRARIES})

add_executable(multi_ecbf_node src/nodes/multi_ecbf_node.cpp)
add_dependencies(multi_ecbf_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_ecbf_node
  multi_ecbf_controller ${catkin_LIBRARIES})

install(TARGETS pid_controller asm_controller rsb_controller ec_controller aec_controller ecbf_controller multi_ecbf_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS pid_node asm_node rsb_node ec_node aec_node ecbf_node multi_ecbf_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)


