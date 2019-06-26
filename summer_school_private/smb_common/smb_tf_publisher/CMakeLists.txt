cmake_minimum_required(VERSION 2.8.3)
project(smb_tf_publisher)

# Activate C++11.
set(CMAKE_CXX_FLAGS "-std=c++11")

set(PACKAGE_DEPENDENCIES
      any_node
      any_msgs
      tf
      robot_state_publisher
      smb_msgs
      cosmo_ros
      smb_description
      smb_description_ros
      message_logger
)

find_package(catkin 
    REQUIRED COMPONENTS
      ${PACKAGE_DEPENDENCIES}
)

catkin_package(
 INCLUDE_DIRS include 
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS
      ${PACKAGE_DEPENDENCIES}
 DEPENDS 
 )
 
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
    src/SmbTfPublisher.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
) 

add_executable(${PROJECT_NAME}_node 
  src/smb_tf_publisher_node.cpp
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_NAME}
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