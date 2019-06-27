cmake_minimum_required(VERSION 2.8.3)
project(rocoma_rqt)

# Set compiler flags
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror ${CMAKE_CXX_FLAGS} ")

find_package(catkin REQUIRED COMPONENTS
    rocoma_msgs
    roscpp
    rqt_gui
    rqt_gui_cpp
    message_logger

)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
  find_package(Qt5OpenGL REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
  include(${QT_USE_FILE})
endif()

find_package(OpenGL REQUIRED)

catkin_python_setup()



###########
# sources #
###########
set(rqt_controlmanager_SRCS
  src/rocoma_rqt/ControlManagerPlugin.cpp
  src/rocoma_rqt/WorkerThreadSetMode.cpp
)
###########



###########
# headers #
###########
set(rqt_controlmanager_HDRS
  include/${PROJECT_NAME}/ControlManagerPlugin.hpp
  include/${PROJECT_NAME}/WorkerThreadSetMode.h
)
###########



############
# ui files #
############
set(rqt_controlmanager_UIS
    resource/rocoma_rqt_plugin.ui
)
############



set(rqt_controlmanager_INCLUDE_DIRECTORIES
    include
    ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
    LIBRARIES rocoma_rqt
    CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp rocoma_msgs message_logger
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_controlmanager_MOCS ${rqt_controlmanager_HDRS})
  qt5_wrap_ui(rqt_controlmanager_UIS_H ${rqt_controlmanager_UIS})
else()
  qt4_wrap_cpp(rqt_controlmanager_MOCS ${rqt_controlmanager_HDRS})
  qt4_wrap_ui(rqt_controlmanager_UIS_H ${rqt_controlmanager_UIS})
endif()

include_directories(
    ${rqt_controlmanager_INCLUDE_DIRECTORIES}
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    ${rqt_controlmanager_SRCS}
    ${rqt_controlmanager_HDRS}
    ${rqt_controlmanager_MOCS}
    ${rqt_controlmanager_UIS_H}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OPENGL_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${OPENGL_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTOPENGL_LIBRARY})
endif()

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

#############
## Install ##
#############

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rocoma_rqt
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)