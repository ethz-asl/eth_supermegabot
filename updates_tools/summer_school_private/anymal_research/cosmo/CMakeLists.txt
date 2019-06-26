cmake_minimum_required(VERSION 2.8.3)
project(cosmo)

find_package(Boost REQUIRED COMPONENTS chrono)

find_package(catkin REQUIRED COMPONENTS
  message_logger
  any_measurements
  any_worker
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} pthread rt
  CATKIN_DEPENDS
    message_logger
    any_measurements
    any_worker
  DEPENDS Boost
  CFG_EXTRAS ${PROJECT_NAME}.cmake
)

###########
## Build ##
###########
add_definitions(-std=c++11 -fPIC -Wall -Werror)

#add debug defines
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}.cmake)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# library
add_library(${PROJECT_NAME}
  src/SyncMaster.cpp
  src/memory_tools.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  pthread
  rt
)

#memory cleaner
add_executable(memory_remover
  src/memory_remover.cpp
)

target_link_libraries(memory_remover
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  pthread
  rt
)

#memory info
add_executable(memory_info
  src/memory_info.cpp
)

target_link_libraries(memory_info
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  pthread
  rt
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  #publisher
  add_executable(${PROJECT_NAME}_publisher_node
    test/${PROJECT_NAME}_test_publisher.cpp
  )

  target_link_libraries(${PROJECT_NAME}_publisher_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    pthread
    rt
  )

  #subscriber
  add_executable(${PROJECT_NAME}_subscriber_node
    test/${PROJECT_NAME}_test_subscriber.cpp
  )

  target_link_libraries(${PROJECT_NAME}_subscriber_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    pthread
    rt
  )
endif()


#############
## Install ##
#############
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME} memory_remover memory_info
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )