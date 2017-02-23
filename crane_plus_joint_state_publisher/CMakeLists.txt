cmake_minimum_required(VERSION 2.8.3)
project(crane_plus_joint_state_publisher)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  rospy
  dynamixel_msgs
  sensor_msgs
  )

install(
  PROGRAMS src/joint_state_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
