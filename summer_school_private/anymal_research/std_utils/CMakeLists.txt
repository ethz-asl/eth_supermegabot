# Project configuration
cmake_minimum_required (VERSION 2.8)

project(std_utils)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
)
# Find Boost
find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  DEPENDS
    Boost
)

###########
## Build ##
###########
add_definitions(--std=c++11)
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)
add_executable(ctm_example src/containers/CompileTimeMapExample.cpp)
add_dependencies(ctm_example ${catkin_EXPORTED_TARGETS})

add_executable(macro_example src/macros/macro_example.cpp)
add_dependencies(macro_example ${catkin_EXPORTED_TARGETS})

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  # Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/containers/CompileTimeMapTest.cpp
    test/containers/CompileTimeSetTest.cpp
    test/containers/CompileTimeKeysTest.cpp
    test/containers/EnumArrayTest.cpp
    test/containers/KeyArrayTest.cpp
    test/containers/MultiKeyContainerTest.cpp
    test/containers/SmartPointerContainer.cpp
#    test/timers/ChronoTimerTest.cpp
    test/remove_first_if_test.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )
endif()

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
