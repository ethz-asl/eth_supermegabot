cmake_minimum_required(VERSION 3.1.3)
project(yaml_tools)

set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  message_logger
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    yaml-cpp
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  src/${PROJECT_NAME}/internal/node_to_ordered_string.cpp
  src/${PROJECT_NAME}/internal/scalar_to_string.cpp
  src/${PROJECT_NAME}/YamlNode.cpp
)
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(
  format_file
  src/format_file.cpp
)
add_dependencies(
  format_file
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  format_file
  ${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME} format_file
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED
    COMPONENTS
      ${CATKIN_PACKAGE_DEPENDENCIES}
  )
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )
  catkin_add_gtest(
    test_${PROJECT_NAME}
    test/main.cpp
    test/TestYamlNodeEmpty.cpp
    test/TestYamlNodeScalar.cpp
    test/TestYamlNodeScalarBoolFalse.cpp
    test/TestYamlNodeScalarBoolTrue.cpp
    test/TestYamlNodeScalarInteger.cpp
    test/TestYamlNodeScalarDoubleNonInteger.cpp
    test/TestYamlNodeScalarDoubleInteger.cpp
    test/TestYamlNodeScalarDoubleScientific.cpp
    test/TestYamlNodeScalarDoubleInfPos.cpp
    test/TestYamlNodeScalarDoubleInfNeg.cpp
    test/TestYamlNodeScalarDoubleNaN.cpp
    test/TestYamlNodeScalarStringSingleWord.cpp
    test/TestYamlNodeScalarStringMultipleWords.cpp
    test/TestYamlNodeScalarStringEmpty.cpp
    test/TestYamlNodeScalarStringSpecialCharacters.cpp
    test/TestYamlNodeMap.cpp
    test/TestYamlNodeSequence.cpp
    test/TestYamlNodeNested.cpp
  )
endif()

if(TARGET test_${PROJECT_NAME})
  target_link_libraries(
    test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
endif()

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_clang_tooling(
      TARGET
          ${PROJECT_NAME}
          format_file
          test_${PROJECT_NAME}
      SOURCE_DIRS
          ${CMAKE_CURRENT_SOURCE_DIR}/src
          ${CMAKE_CURRENT_SOURCE_DIR}/include
          ${CMAKE_CURRENT_SOURCE_DIR}/test
      CT_HEADER_DIRS
          ${CMAKE_CURRENT_SOURCE_DIR}/include
      CF_FIX
  )
endif(cmake_clang_tools_FOUND)
