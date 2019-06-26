cmake_minimum_required(VERSION 2.8.3)

project(smb_joystick_controller)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roco
  smb_roco
  yaml_tools
  roco_ros
  rocoma_plugin
  any_measurements
  any_measurements_ros
)

# Flags
add_definitions(-std=c++11)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roco
    smb_roco
    yaml_tools
    roco_ros
    rocoma_plugin
    any_measurements
    any_measurements_ros
)

include_directories(include
  ${rocoma_plugin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/SmbJoystickController.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

add_library(${PROJECT_NAME}_plugin
  src/${PROJECT_NAME}_plugin.cpp
)

target_link_libraries(${PROJECT_NAME}_plugin
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${rocoma_plugin_LIBRARIES}
)

install(FILES smb_joystick_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        )