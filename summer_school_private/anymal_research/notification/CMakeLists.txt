cmake_minimum_required(VERSION 2.8.3)
project(notification)

add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
    notification_msgs
    roscpp
)

catkin_package(
  CATKIN_DEPENDS
    notification_msgs
    roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    notification
  CATKIN_DEPENDS
    notification_msgs
#  DEPENDS
#    system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(notification
  src/Notification.cpp
  src/NotificationPublisher.cpp
  src/NotificationSubscriber.cpp
)

add_dependencies(notification
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(notification
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
