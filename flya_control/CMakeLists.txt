cmake_minimum_required(VERSION 2.8.3)
project(flya_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS std_msgs
             geometry_msgs
             mav_msgs
             nav_msgs
             roscpp
             sensor_msgs
             cmake_modules
             rotors_control
)

find_package(Eigen3 REQUIRED)

add_message_files(FILES Waypoint.msg)
generate_messages()

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES lee_position_controller roll_pitch_yawrate_thrust_controller
  CATKIN_DEPENDS std_msgs geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs rotors_control
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/library/quaternion_controller.cpp
  src/library/waypoint_controller.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

add_executable(quaternion_controller_node src/nodes/quaternion_controller_node.cpp)
target_link_libraries(quaternion_controller_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(quaternion_controller_node ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_controller_node src/nodes/waypoint_controller_node.cpp)
target_link_libraries(waypoint_controller_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(quaternion_controller_node ${catkin_EXPORTED_TARGETS})

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${PROJECT_NAME} quaternion_controller_node waypoint_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
