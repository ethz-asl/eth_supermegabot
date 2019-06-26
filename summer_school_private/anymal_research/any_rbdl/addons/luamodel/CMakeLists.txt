add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMake)
find_package(Lua51 REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include/any_rbdl
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${LUA_INCLUDE_DIR}
)

set(LUAMODEL_SOURCES
  luamodel.cc
  luatables.cc
)

# Library
add_library(any_rbdl_luamodel
  ${LUAMODEL_SOURCES}
)

set_target_properties(any_rbdl_luamodel PROPERTIES
  VERSION ${RBDL_VERSION}
  SOVERSION ${RBDL_SO_VERSION}
)

target_link_libraries(any_rbdl_luamodel
  any_rbdl
  ${LUA_LIBRARIES}
)

# Executable
add_executable(any_rbdl_luamodel_util
  rbdl_luamodel_util.cc
)

target_link_libraries(any_rbdl_luamodel_util
  any_rbdl_luamodel
  any_rbdl
)

# Install
install(TARGETS any_rbdl_luamodel any_rbdl_luamodel_util
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/any_rbdl/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
