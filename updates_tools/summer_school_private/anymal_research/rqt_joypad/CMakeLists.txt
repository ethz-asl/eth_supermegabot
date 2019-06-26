cmake_minimum_required(VERSION 2.8.3)
project(rqt_joypad)

# Set compiler flags
add_definitions(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  joy_manager_msgs
  joystick_label
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

###########
# sources #
###########
set(rqt_joypad_SRCS
  src/rqt_joypad/JoypadPlugin.cpp
)
###########



###########
# headers #
###########
set(rqt_joypad_HDRS
  include/rqt_joypad/JoypadPlugin.hpp
)
###########



############
# ui files #
############
set(rqt_joypad_UIS
  resource/joypad_plugin.ui
)
############


set(rqt_joypad_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  LIBRARIES rqt_joypad
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp joy_manager_msgs joystick_label
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_joypad_MOCS ${rqt_joypad_HDRS})
  qt5_wrap_ui(rqt_joypad_UIS_H ${rqt_joypad_UIS})
else()
  qt4_wrap_cpp(rqt_joypad_MOCS ${rqt_joypad_HDRS})
  qt4_wrap_ui(rqt_joypad_UIS_H ${rqt_joypad_UIS})
endif()

include_directories(
  ${rqt_joypad_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)


add_library(rqt_joypad
  ${rqt_joypad_SRCS}
  ${rqt_joypad_HDRS}
  ${rqt_joypad_MOCS}
  ${rqt_joypad_UIS_H}
)

add_dependencies(rqt_joypad
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(rqt_joypad
  ${catkin_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTOPENGL_LIBRARY})
endif()


find_package(class_loader)
class_loader_hide_library_symbols(rqt_joypad)


install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS rqt_joypad
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_joypad
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
