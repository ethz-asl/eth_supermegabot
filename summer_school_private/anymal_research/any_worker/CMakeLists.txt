cmake_minimum_required(VERSION 2.8.3)
project(any_worker)

find_package(catkin REQUIRED COMPONENTS
  message_logger
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} pthread
  CATKIN_DEPENDS message_logger
)

###########
## Build ##
###########
add_definitions(-std=c++11 -Wall -Werror)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  src/Rate.cpp
  src/Worker.cpp
  src/WorkerManager.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

##########
## Test ##
##########
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    ${PROJECT_NAME}_test
    test/${PROJECT_NAME}_test.cpp
    test/RateTest.cpp
  )
endif()

if(TARGET ${PROJECT_NAME}_test)
  target_link_libraries(
    ${PROJECT_NAME}_test
    ${PROJECT_NAME}
  )
endif()

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
