cmake_minimum_required(VERSION 2.8.3)

project(smb_failproof_controller)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roco
  smb_roco
  rocoma_plugin
)

# Flags
add_definitions(-std=c++11)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roco
    smb_roco
    rocoma_plugin
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${rocoma_plugin_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME}
  src/SmbFailproofController.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_plugin
  src/${PROJECT_NAME}_plugin.cpp
)

target_link_libraries(${PROJECT_NAME}_plugin
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${rocoma_plugin_LIBRARIES}
)

catkin_add_gtest(test_smb_failproof_controller
  test/test_smb_failproof_controller.cpp
  )
add_dependencies(test_smb_failproof_controller
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(test_smb_failproof_controller
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  )


#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES smb_failproof_controller_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )