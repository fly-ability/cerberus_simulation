cmake_minimum_required(VERSION 2.8.3)
project(flya_description)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  urdf
  controller_manager
  joint_state_controller
  robot_state_publisher
  #gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package(

)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


##################
include_directories(include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS ${GAZEBO_CXX_FLAGS})
