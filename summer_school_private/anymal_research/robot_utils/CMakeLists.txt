# Project configuration
cmake_minimum_required (VERSION 2.8)

project(robot_utils)

# Find Catkin
find_package(catkin REQUIRED COMPONENTS
  message_logger
  parameter_handler
  std_utils
  cmake_modules
)

# Attempt to find catkinized kindr
find_package(kindr QUIET)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(kindr kindr REQUIRED)
endif()

find_package(Eigen3 REQUIRED)
find_package(TinyXML REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    message_logger
    parameter_handler
    std_utils
  DEPENDS
    kindr
    TinyXML
)

###########
## Build ##
###########

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_definitions(--std=c++11)

include_directories (
  include
  ${kindr_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  # Controllers
  src/controllers/DiscretePidEigen.cpp
  src/controllers/PidEigen.cpp
  src/controllers/PIDController.cpp
  src/controllers/PIDControllerFeedForward.cpp
  src/controllers/PidControllerQuaternion.cpp

  # Filters
  src/filters/BinaryChatteringCompensator.cpp
  src/filters/FilterButter.cpp
  src/filters/RateLimiter.cpp

  # Function Approximators
  src/function_approximators/dynamicMovementPrimitive/DiscreteMovementPrimitive.cpp
  src/function_approximators/dynamicMovementPrimitive/DmpParameters.cpp
  src/function_approximators/dynamicMovementPrimitive/GaussianKernel.cpp
  src/function_approximators/dynamicMovementPrimitive/GaussianKernelApproximator.cpp
  src/function_approximators/dynamicMovementPrimitive/GaussianKernelJumpPropagator.cpp
  src/function_approximators/dynamicMovementPrimitive/TimelessGaussianKernelApproximator.cpp
  src/function_approximators/dynamicMovementPrimitive/TransformationSystem.cpp

  src/function_approximators/gaussianBaseTrajectory/GaussianBaseTrajectory.cpp

  src/function_approximators/polyharmonicSplines/RBF1D.cpp
  src/function_approximators/polyharmonicSplines/PeriodicRBF1DC3.cpp
  src/function_approximators/polyharmonicSplines/PeriodicRBF1DC1.cpp
  src/function_approximators/polyharmonicSplines/BoundedRBF1D.cpp

  # Function Generators
  src/function_generators/FunctionGeneratorBase.cpp
  src/function_generators/FctLinChirp.cpp
  src/function_generators/FctLogChirp.cpp

  # Geometry
  src/geometry/common/geometry_utils.cpp
  src/geometry/Polygon.cpp
  src/geometry/Line.cpp
  src/geometry/Triangle.cpp
  src/geometry/Tetragon.cpp

  # Interpolators
  src/interpolators/BasicFifthOrderInterpolation.cpp
  src/interpolators/LinearInterpolation.cpp

  # Kinematics
  src/kinematics/kinematics.cpp

  # Math
  src/math/LinearAlgebra.cpp
  src/math/Stochastics.cpp
  src/math/Transformations.cpp

  # Sensors
  src/sensors/Joystick.cpp
  src/sensors/GNSS.cpp

  # Surfaces
  src/surfaces/Surface.cpp
  src/surfaces/Ellipsoid.cpp

  src/contact_detectors/ContactDetectorBase.cpp
  src/contact_detectors/ContactDetectorThresholding.cpp

  src/force_calibrators/ForceCalibratorBase.cpp
  src/force_calibrators/AverageForceCalibrator.cpp
  src/force_calibrators/NoneForceCalibrator.cpp

  src/physical_definitions.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${TinyXML_LIBRARIES}
)
# Add clang tooling
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_clang_tooling(
    TARGET ${PROJECT_NAME}
    DISABLE_CLANG_TIDY
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/interpolators ${CMAKE_CURRENT_SOURCE_DIR}/include/interpolators ${CMAKE_CURRENT_SOURCE_DIR}/test/interpolators
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
  )
endif(cmake_clang_tools_FOUND)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  # Add gtest based cpp test target and link libraries
  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_main.cpp
    test/schedule/ScheduleTest.cpp
    test/geometry/GeometryTest.cpp
    test/math/LinearAlgebraTest.cpp
    test/math/TransformationsTest.cpp
    test/math/statistics_test.cpp
    test/eigen/EigenExtensionsTest.cpp
    test/controllers/PidEigenTest.cpp
    test/controllers/DiscretePidEigenTest.cpp
    test/filters/ContinuousTimeTransferFunctionTest.cpp
    test/filters/FilterMovAvgTest.cpp
    test/filters/FirstOrderFilterTest.cpp
    test/filters/FirstOrderDerivativeTest.cpp
    test/force_calibrators/AverageForceCalibratorTest.cpp
    test/interpolators/InterpolationTest.cpp
    test/interpolators/InterpolatorTest.cpp
    test/kinematics/kinematicsTest.cpp
    test/sensors/GNSSTest.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )
  if(TARGET ${PROJECT_NAME}-test)
    target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
  endif()
endif()

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
