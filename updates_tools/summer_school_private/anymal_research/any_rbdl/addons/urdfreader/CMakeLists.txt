add_definitions(-std=c++11)

# Find packages
find_package(catkin REQUIRED)
find_package(urdfdom REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include/any_rbdl
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${urdfdom_INCLUDE_DIRS}
)

# Source files
set(URDFREADER_SOURCES
  urdfreader.cc
)

# Library
add_library(any_rbdl_urdfreader
  ${URDFREADER_SOURCES}
)

set_target_properties(any_rbdl_urdfreader PROPERTIES
  VERSION ${ANY_RBDL_VERSION}
  SOVERSION ${ANY_RBDL_SO_VERSION}
)

target_link_libraries(any_rbdl_urdfreader
  any_rbdl
  ${urdfdom_LIBRARIES}
)

# Executable
add_executable(any_rbdl_urdfreader_util
  rbdl_urdfreader_util.cc
)

target_link_libraries(any_rbdl_urdfreader_util
  any_rbdl_urdfreader
  any_rbdl
)

# Install
install(TARGETS any_rbdl_urdfreader any_rbdl_urdfreader_util
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/any_rbdl/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
