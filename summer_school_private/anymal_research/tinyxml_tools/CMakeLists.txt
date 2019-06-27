cmake_minimum_required(VERSION 2.8.3)
project(tinyxml_tools)

add_definitions(-std=c++11 -Wall -Wextra -Wpedantic)

# Generate compile_commands.json
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
  COMPONENTS
    message_logger
)

# Attempt to find package-based tinyxml
find_package(PkgConfig REQUIRED)
pkg_check_modules(tinyxml tinyxml REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    message_logger
  DEPENDS
    tinyxml
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${tinyxml_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}
  src/write_helpers.cpp
  src/read_helpers.cpp
  src/DocumentHandleXML.cpp
)
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${tinyxml_LIBRARIES}
)

# Add clang tooling
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_clang_tooling(
    TARGET ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
  )
endif(cmake_clang_tools_FOUND)

#############
## Testing ##
#############
catkin_add_gtest(test_${PROJECT_NAME}
  test/test_main.cpp
  test/writeXMLTests.cpp
  test/loadXMLTests.cpp
  test/documentHandleXMLTests.cpp
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
)
if(TARGET test_${PROJECT_NAME})
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
endif()

install(DIRECTORY test/xml_includes # For unit tests of tinyxml_tools_ros
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
