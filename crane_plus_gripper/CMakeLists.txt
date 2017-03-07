cmake_minimum_required(VERSION 2.8.3)
project(crane_plus_gripper)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  rospy
  control_msgs
  )

install(
  PROGRAMS src/gripper_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
