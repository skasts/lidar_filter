cmake_minimum_required(VERSION 3.0.2)
project(lidar_filter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  roslaunch
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  scripts/lidar_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)