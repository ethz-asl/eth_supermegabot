cmake_minimum_required(VERSION 2.8.3)
project(rviz_world_loader)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()

install(PROGRAMS
  bin/rviz_world_loader/rviz_world_loader.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.launch"
)
