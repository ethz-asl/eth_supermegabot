cmake_minimum_required(VERSION 2.8.3)
project(confusion)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wfatal-errors")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    roslib
    std_msgs
    geometry_msgs
    sensor_msgs
    message_generation
    image_transport
    cv_bridge
    tf)

find_package(Eigen3 REQUIRED)
find_package(Ceres QUIET)
find_package(OpenCV QUIET)

add_message_files(FILES
  TagMsg.msg
  TagArray.msg
  OdomWithState.msg)
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs)

if (Ceres_FOUND)
  if (OpenCV_FOUND)
    add_definitions("-DOPENCV_FOUND=1")

    catkin_package(
        INCLUDE_DIRS include example
        LIBRARIES ConFusor BatchFusor MeasurementManager Diagram ImuPropagator april_tag_detector Logger april_tag_module ExternalReferenceTrackingModule
        CATKIN_DEPENDS roscpp image_transport cv_bridge
        DEPENDS CERES OpenCV
    )
    include_directories(${OpenCV_INCLUDE_DIRS})

    add_library(Diagram src/Diagram.cpp)
    target_link_libraries(Diagram ${OpenCV_LIBRARIES})
  else ()
    message(WARNING "OpenCV not found. Will not be able to use the MHE Diagram drawing functionality or the TagTracker example.")
    catkin_package(
        INCLUDE_DIRS include example
        LIBRARIES ConFusor BatchFusor MeasurementManager Diagram ImuPropagator Logger ExternalReferenceTrackingModule
        CATKIN_DEPENDS roscpp
        DEPENDS CERES
    )
  endif ()

  include_directories(
      include
      example
    include/confusion/modules/apriltag/external
      test
      ${catkin_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIR}
      ${CERES_INCLUDE_DIR})

  add_library(ConFusor src/ConFusor.cpp)
  target_link_libraries(ConFusor ${CERES_LIBRARIES})

  add_library(BatchFusor src/BatchFusor.cpp)
  target_link_libraries(BatchFusor ${CERES_LIBRARIES})

  add_library(MeasurementManager src/MeasurementManager.cpp)
  add_library(ImuPropagator src/ImuPropagator.cpp)
  add_library(Logger src/Logger.cpp)

  add_library(ExternalReferenceTrackingModule src/modules/ExternalReferenceTrackingModule.cpp)
  add_dependencies(ExternalReferenceTrackingModule ${confusion_EXPORTED_TARGETS})

  catkin_add_gtest(parameterizationTest test/parameterizationTest.cpp)
  target_link_libraries(parameterizationTest ${catkin_LIBRARIES} ${CERES_LIBRARIES})

  catkin_add_gtest(measurementManagerTest test/MeasurementManagerTest.cpp)
  target_link_libraries(measurementManagerTest ConFusor MeasurementManager ${catkin_LIBRARIES} ${CERES_LIBRARIES})

  catkin_add_gtest(ImuChainTest test/ImuChainTest.cpp)
  target_link_libraries(ImuChainTest ConFusor MeasurementManager ${catkin_LIBRARIES} ${CERES_LIBRARIES})

  catkin_add_gtest(ConFusorTest test/ConFusorTest.cpp)
  target_link_libraries(ConFusorTest BatchFusor ConFusor MeasurementManager Diagram Logger ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CERES_LIBRARIES})
  add_dependencies(ConFusorTest ${confusion_EXPORTED_TARGETS})

  catkin_add_gtest(PriorCostTest test/PriorCostTest.cpp)
  target_link_libraries(PriorCostTest ${catkin_LIBRARIES} ${CERES_LIBRARIES})

  catkin_add_gtest(marginalizationTest test/marginalizationTest.cpp)
  target_link_libraries(marginalizationTest ${catkin_LIBRARIES} ${CERES_LIBRARIES})

  catkin_add_gtest(distancesTest test/distancesTest.cpp)
  target_link_libraries(distancesTest ${catkin_LIBRARIES} ${CERES_LIBRARIES})

  if (OpenCV_FOUND)
    add_library(apriltag-common
      include/confusion/modules/apriltag/external/common/g2d.c
      include/confusion/modules/apriltag/external/common/getopt.c
      include/confusion/modules/apriltag/external/common/homography.c
      include/confusion/modules/apriltag/external/common/image_f32.c
      include/confusion/modules/apriltag/external/common/image_u8.c
      include/confusion/modules/apriltag/external/common/image_u8x3.c
      include/confusion/modules/apriltag/external/common/image_u8x4.c
      include/confusion/modules/apriltag/external/common/matd.c
      include/confusion/modules/apriltag/external/common/pam.c
      include/confusion/modules/apriltag/external/common/pjpeg.c
      include/confusion/modules/apriltag/external/common/pjpeg-idct.c
      include/confusion/modules/apriltag/external/common/pnm.c
      include/confusion/modules/apriltag/external/common/string_util.c
      include/confusion/modules/apriltag/external/common/svd22.c
      include/confusion/modules/apriltag/external/common/time_util.c
      include/confusion/modules/apriltag/external/common/unionfind.c
      include/confusion/modules/apriltag/external/common/workerpool.c
      include/confusion/modules/apriltag/external/common/zarray.c
      include/confusion/modules/apriltag/external/common/zhash.c
      include/confusion/modules/apriltag/external/common/zmaxheap.c)
    target_link_libraries(apriltag-common pthread m)
    set_target_properties(apriltag-common PROPERTIES COMPILE_FLAGS "-std=gnu99 -Wall -Wno-unused-parameter -Wno-unused-function -pthread -O4 -fno-strict-overflow")

    add_library(apriltag
      include/confusion/modules/apriltag/external/apriltag.c
      include/confusion/modules/apriltag/external/apriltag_quad_thresh.c
      include/confusion/modules/apriltag/external/tag16h5.c
      include/confusion/modules/apriltag/external/tag25h9.c
      include/confusion/modules/apriltag/external/tag25h7.c
      include/confusion/modules/apriltag/external/tag36h10.c
      include/confusion/modules/apriltag/external/tag36artoolkit.c
      include/confusion/modules/apriltag/external/tag36h11.c)
    target_link_libraries(apriltag
        apriltag-common ${catkin_LIBRARIES})
    set_target_properties(apriltag PROPERTIES COMPILE_FLAGS "-std=gnu99 -Wall -Wno-unused-parameter -Wno-unused-function -pthread -O4 -fno-strict-overflow")

    add_library(april_tag_detector include/confusion/modules/apriltag/external/april_tag_detector.cpp)
    target_link_libraries(april_tag_detector apriltag ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
    add_dependencies(april_tag_detector ${confusion_EXPORTED_TARGETS})

    add_library(april_tag_module src/modules/AprilTagModule.cpp)
    target_link_libraries(april_tag_module april_tag_detector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

    add_executable(tag_tracker_node example/src/tag_tracker_node.cpp)
    target_link_libraries(tag_tracker_node april_tag_module ConFusor MeasurementManager Diagram BatchFusor ImuPropagator Logger ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBRARIES})
    add_dependencies(tag_tracker_node ${confusion_EXPORTED_TARGETS})
  endif ()

else ()
  message(WARNING "Ceres Solver not found. Will only build the ConFusion utilities.")

  catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ImuPropagator
      CATKIN_DEPENDS roscpp
  )

  include_directories(
      include
      test
      ${catkin_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIR}
  )

  add_library(ImuPropagator src/ImuPropagator.cpp)

endif ()


add_executable(visualize_pose src/utilities/visualizePose.cpp)
target_link_libraries(visualize_pose ${catkin_LIBRARIES})