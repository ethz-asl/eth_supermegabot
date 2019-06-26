cmake_minimum_required(VERSION 2.8.3)
if(POLICY CMP0043)
  cmake_policy(SET CMP0043 NEW)
endif()

project(rqt_parameters)

# Set compiler flags
add_definitions(-std=c++11 -Wall -Werror)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rqt_gui
    rqt_gui_cpp
    parameter_handler_msgs
    std_srvs
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
set(${PROJECT_NAME}_SRCS
    src/${PROJECT_NAME}/ParametersPlugin.cpp
)
###########



###########
# headers #
###########
set(${PROJECT_NAME}_HDRS
    include/${PROJECT_NAME}/ParametersPlugin.hpp
    include/${PROJECT_NAME}/MatrixSpinBox.hpp
    include/${PROJECT_NAME}/ParameterBase.hpp
    include/${PROJECT_NAME}/ParameterFloat64Matrix.hpp
    include/${PROJECT_NAME}/ParameterInt32Matrix.hpp
    include/${PROJECT_NAME}/multiarray_helpers.hpp
)
###########



############
# ui files #
############
set(${PROJECT_NAME}_UIS
    resource/parameters_plugin.ui
)
############

set(ui_INCLUDE_DIR
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include
  ${ui_INCLUDE_DIR}/..
)

if(NOT EXISTS ${ui_INCLUDE_DIR})
  file(MAKE_DIRECTORY ${ui_INCLUDE_DIR})
endif()

catkin_package(
    INCLUDE_DIRS
      ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp parameter_handler_msgs std_srvs
)

include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
  )


if ("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  include_directories(${Qt5Widgets_INCLUDE_DIRS})
  add_definitions(${Qt5Widgets_DEFINITIONS})
  qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
else()
  qt4_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
endif()

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${ui_INCLUDE_DIR})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
else()
  qt4_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})
endif()
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

add_library(rqt_parameters
    ${rqt_parameters_SRCS}
    ${rqt_parameters_MOCS}
    ${rqt_parameters_UIS_H}
)

add_dependencies(rqt_parameters
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(rqt_parameters
    ${catkin_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_use_modules(${PROJECT_NAME} Widgets)
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()


find_package(class_loader)
class_loader_hide_library_symbols(rqt_parameters)


install(FILES plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS rqt_parameters
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    launch
    params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
