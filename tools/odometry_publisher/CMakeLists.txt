cmake_minimum_required(VERSION 2.8.3)
project(odometry_publisher)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
add_definitions(-std=c++11)

# Add executables.
cs_add_executable(odometry_publisher src/odometry_publisher.cc)

############
## EXPORT ##
############
cs_install()
cs_export()
