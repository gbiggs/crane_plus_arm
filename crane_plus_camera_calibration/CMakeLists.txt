cmake_minimum_required(VERSION 2.8.3)
project(crane_plus_camera_calibration)

find_package(
  catkin REQUIRED
  cmake_modules
  cv_bridge
  image_geometry
  image_transport
  pcl_ros
  roscpp
  tf
  visualization_msgs)
find_package(Boost REQUIRED system filesystem)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  )

catkin_package(
  DEPENDS
  cv_bridge
  image_geometry
  image_transport
  pcl_ros
  roscpp
  tf
  visualization_msgs
  )

include_directories(
  include SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

add_executable(
  calibrate_camera_checkerboard
  src/calibrate_camera_checkerboard.cpp
  src/detect_calibration_pattern.cpp
  )
target_link_libraries(
  calibrate_camera_checkerboard
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

install(
  TARGETS calibrate_camera_checkerboard
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
