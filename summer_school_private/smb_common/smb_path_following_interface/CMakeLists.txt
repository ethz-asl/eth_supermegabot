cmake_minimum_required(VERSION 2.8.3)
project(smb_path_following_interface)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roslib
  ocs2_comm_interfaces
  ocs2_core
  ocs2_robotic_tools
  ocs2_slq
  any_measurements
  kindr
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  )

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roslib
    ocs2_comm_interfaces
    ocs2_core
    ocs2_robotic_tools
    ocs2_slq
    any_measurements
    kindr
  DEPENDS
    Boost
)

###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_explicit_instantiations
  src/SmbPathFollowingExplicitTemplateInstantiations.cpp)

add_dependencies(${PROJECT_NAME}_explicit_instantiations ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_explicit_instantiations
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
  )

add_library(${PROJECT_NAME}
  src/SmbPathFollowingSystemDynamics.cpp
  src/SmbPathFollowingCost.cpp
  src/SmbPathFollowingOperatingPoint.cpp
  src/SmbPathFollowingInterface.cpp
  src/SmbPathFollowingConversions.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_explicit_instantiations
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  dl
  )


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

catkin_add_gtest(smb_path_following_interface_test
  test/TestSmbPathFollowingInterface.cpp
  )
add_dependencies(smb_path_following_interface_test
  ${catkin_EXPORTED_TARGETS}
  )
target_link_libraries(smb_path_following_interface_test
  ${PROJECT_NAME}_explicit_instantiations
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
