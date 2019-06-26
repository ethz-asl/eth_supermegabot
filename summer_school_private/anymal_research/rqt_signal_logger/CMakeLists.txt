cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0043)
  cmake_policy(SET CMP0043 NEW)
endif()

project(rqt_signal_logger)

# Set compiler flags
add_definitions(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rqt_gui
    rqt_gui_cpp
    signal_logger_msgs
    signal_logger_core
    std_srvs
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5 COMPONENTS Core Widgets Designer REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

###########
# sources #
###########
set(rqt_signal_logger_SRCS
  src/rqt_signal_logger/SignalLoggerPlugin.cpp
  src/rqt_signal_logger/BufferIndicator.cpp
)
###########



###########
# headers #
###########
set(rqt_signal_logger_HDRS
  include/rqt_signal_logger/SignalLoggerPlugin.hpp
  include/rqt_signal_logger/LogElement.hpp
  include/rqt_signal_logger/BufferIndicator.hpp
)
###########



############
# ui files #
############
set(rqt_signal_logger_UIS
  resource/SignalLoggerPlugin.ui
  resource/SignalLoggerConfigure.ui
  resource/SignalLoggerVariables.ui
)
############



set(rqt_signal_logger_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    rqt_signal_logger
    yaml-cpp
  CATKIN_DEPENDS
    roscpp
    rqt_gui
    rqt_gui_cpp
    signal_logger_msgs
    signal_logger_core
    std_srvs
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_signal_logger_MOCS ${rqt_signal_logger_HDRS})
  qt5_wrap_ui(rqt_signal_logger_UIS_H ${rqt_signal_logger_UIS})
else()
  qt4_wrap_cpp(rqt_signal_logger_MOCS ${rqt_signal_logger_HDRS})
  qt4_wrap_ui(rqt_signal_logger_UIS_H ${rqt_signal_logger_UIS})
endif()

include_directories(
    ${rqt_signal_logger_INCLUDE_DIRECTORIES}
    ${catkin_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Designer_INCLUDE_DIRS}
)


add_library(rqt_signal_logger
    ${rqt_signal_logger_SRCS}
    ${rqt_signal_logger_HDRS}
    ${rqt_signal_logger_MOCS}
    ${rqt_signal_logger_UIS_H}
)

add_dependencies(rqt_signal_logger ${catkin_EXPORTED_TARGETS})

target_link_libraries(rqt_signal_logger
    ${catkin_LIBRARIES}
    yaml-cpp
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_use_modules(${PROJECT_NAME} Widgets)
  target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

find_package(class_loader)
class_loader_hide_library_symbols(rqt_signal_logger)


install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(TARGETS rqt_signal_logger
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_signal_logger
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING
  PATTERN "*.launch"
)

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
  FILES_MATCHING
  PATTERN "*.yaml"
)
