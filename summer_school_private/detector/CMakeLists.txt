cmake_minimum_required(VERSION 2.8.3)
project(detector)

#################
#    DARKNET    #
#################
add_subdirectory(darknet)

#################
# CATKIN SIMPLE #
#################
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#################
#  EXECUTABLES  #
#################
add_definitions(-DOPENCV)

cs_add_executable(detector
    src/detector.cc
    src/utils.cc)
target_link_libraries(detector dark)

#################
#     EXPORT    #
#################
cs_install()
cs_export()
