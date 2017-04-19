cmake_minimum_required(VERSION 2.8.3)
project(crane_plus_move_to_pose)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  rospy
  )

install(
  PROGRAMS script/move_to_pose.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
