cmake_minimum_required(VERSION 2.8.3)
project(smb_driver)
add_definitions("-std=c++11 -Wfatal-errors")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules)
        
find_package(Boost REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -pthread")

catkin_package(
  INCLUDE_DIRS include ${BOOST_INCLUDE_DIRS}
  LIBRARIES serialMB
)

#find_package(Eigen REQUIRED)

include_directories(
    include
    test
    ${catkin_INCLUDE_DIRS}
	${BOOST_INCLUDE_DIRS}
)

set(SERIALRT_SRC
	src/RoboteqDevice.cpp #Use of rt_serial is not supported currently!
    src/SmbController.cpp
	src/auxiliaries/interProcessCommunication.cpp
)
add_library(serialMB ${SERIALRT_SRC})

add_executable(testSerial test/SerialTest.cpp)
target_link_libraries(testSerial serialMB ${catkin_LIBRARIES})

add_executable(stopMB test/StopMB.cpp)
target_link_libraries(stopMB serialMB ${catkin_LIBRARIES})

add_executable(testController test/ControllerTest.cpp)
target_link_libraries(testController serialMB ${catkin_LIBRARIES})

#add_executable(TrqModeTest test/TrqModeTest.cpp)
#target_link_libraries(TrqModeTest serialMB ${catkin_LIBRARIES})

install(TARGETS serialMB
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
