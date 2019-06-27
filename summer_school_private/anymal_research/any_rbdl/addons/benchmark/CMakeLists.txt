add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake)

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/include/any_rbdl
)

# Perform the proper linking
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Options
set(BENCHMARK_SOURCES
  model_generator.cc
  Human36Model.cc
  benchmark.cc
)

add_executable(benchmark
  ${BENCHMARK_SOURCES}
)

set(LIBRARIES any_rbdl)

IF (BUILD_ADDON_LUAMODEL)
  set(LIBRARIES ${LIBRARIES} any_rbdl_luamodel)
ENDIF (BUILD_ADDON_LUAMODEL)

IF (BUILD_ADDON_URDFREADER)
  set(LIBRARIES ${LIBRARIES} any_rbdl_urdfreader)
ENDIF (BUILD_ADDON_URDFREADER)

target_link_libraries(benchmark
  any_rbdl
  ${LIBRARIES}
)
