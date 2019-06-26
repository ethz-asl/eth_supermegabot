cmake_minimum_required(VERSION 2.8.3)
project(rqt_gazebo_interface)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rqt_gui
    rqt_gui_cpp
    std_srvs
    gazebo_msgs
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

###########
# sources #
###########
set(rqt_gazebo_interface_SRCS
  src/rqt_gazebo_interface/GazeboInterfacePlugin.cpp
)
###########



###########
# headers #
###########
set(rqt_gazebo_interface_HDRS
    include/rqt_gazebo_interface/GazeboInterfacePlugin.hpp
)
###########



############
# ui files #
############
set(rqt_gazebo_interface_UIS
    resource/gazebo_interface_plugin.ui
)
############



set(rqt_gazebo_interface_INCLUDE_DIRECTORIES
    include
    ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  LIBRARIES rqt_gazebo_interface
  CATKIN_DEPENDS 
    roscpp 
    rqt_gui 
    rqt_gui_cpp
    std_srvs
    gazebo_msgs
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_gazebo_interface_MOCS ${rqt_gazebo_interface_HDRS})
  qt5_wrap_ui(rqt_gazebo_interface_UIS_H ${rqt_gazebo_interface_UIS})
else()
  qt4_wrap_cpp(rqt_gazebo_interface_MOCS ${rqt_gazebo_interface_HDRS})
  qt4_wrap_ui(rqt_gazebo_interface_UIS_H ${rqt_gazebo_interface_UIS})
endif()


include_directories(
    ${rqt_gazebo_interface_INCLUDE_DIRECTORIES}
    ${catkin_INCLUDE_DIRS}
)


add_library(rqt_gazebo_interface
    ${rqt_gazebo_interface_SRCS}
    ${rqt_gazebo_interface_MOCS}
    ${rqt_gazebo_interface_UIS_H}
)


target_link_libraries(rqt_gazebo_interface
    ${catkin_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()


find_package(class_loader)
class_loader_hide_library_symbols(rqt_gazebo_interface)


install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS rqt_gazebo_interface
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_gazebo_interface
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
