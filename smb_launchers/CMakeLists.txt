cmake_minimum_required(VERSION 2.8.3)
project(smb_launchers)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

############
# BINARIES #
############

##########
# EXPORT #
##########
cs_install()
cs_export()
