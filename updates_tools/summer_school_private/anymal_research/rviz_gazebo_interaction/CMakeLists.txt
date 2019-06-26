cmake_minimum_required(VERSION 2.8.3)
project(rviz_gazebo_interaction)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    gazebo_msgs
    roscpp
    rqt_gui
    rqt_gui_cpp
    rviz
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

set(${PROJECT_NAME}_HDRS_QT
  include/rviz_gazebo_interaction/wrenchTool.hpp
)
set(${PROJECT_NAME}_HDRS
  include/rviz_gazebo_interaction/anyLinkSelectionHandler.hpp
)

set(${PROJECT_NAME}_SRCS
  src/wrenchTool.cpp
  src/anyLinkSelectionHandler.cpp
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    gazebo_msgs
    roscpp
    rqt_gui
    rqt_gui_cpp
    rviz
#  DEPENDS system_lib
)

###########
## Build ##
###########

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS_QT})
else()
  qt4_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS_QT})
endif()

include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_HDRS_QT}
  ${${PROJECT_NAME}_HDRS}
  ${${PROJECT_NAME}_MOCS}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME}
    ${QT_QTCORE_LIBRARY}
    ${QT_QTOPENGL_LIBRARY}
  )
endif()

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
