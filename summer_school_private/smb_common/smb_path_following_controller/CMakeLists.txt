cmake_minimum_required(VERSION 2.8.3)

project(smb_path_following_controller)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roco
  rocoma_plugin
  smb_path_following_interface
  roco_ros
  ocs2_core
  smb_roco
  message_logger
  yaml_tools
  )

# Flags
add_definitions(-std=c++14)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
    roco
    rocoma_plugin
    smb_path_following_interface
    roco_ros
    ocs2_core
    smb_roco
    message_logger
    yaml_tools
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/SmbPathFollowingController.cpp
  )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

add_library(${PROJECT_NAME}_plugin
  src/${PROJECT_NAME}_plugin.cpp
  )

add_dependencies(${PROJECT_NAME}_plugin ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_plugin
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )

install(FILES rocoma_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(TARGETS ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
