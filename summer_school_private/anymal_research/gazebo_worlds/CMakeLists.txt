cmake_minimum_required(VERSION 2.8.3)
project(gazebo_worlds)

find_package(catkin REQUIRED)

catkin_package()

execute_process(
  COMMAND cmake -E create_symlink ${CMAKE_SOURCE_DIR}/meshes
    ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/meshes
  COMMAND cmake -E create_symlink ${CMAKE_SOURCE_DIR}/models
    ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/models
)

install(DIRECTORY models meshes worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
