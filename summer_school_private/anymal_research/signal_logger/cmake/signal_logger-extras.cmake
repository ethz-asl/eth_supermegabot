# Default ros is on
option(SILO_USE_ROS "Use ros in signal_logger?" ON)
if (SILO_USE_ROS)
  MESSAGE(STATUS "Signal logger is using ros!")
	add_definitions(-DSILO_USE_ROS)
else(SILO_USE_ROS)
  MESSAGE(STATUS "Signal logger is not using ros!")
endif(SILO_USE_ROS)
