# Parameter Handler

## Overview

These packages provide an interface for a parameter handler and some default parameter handlers.
A parameter handler is a tool to tune parameters defined in other classes.

[Documentation](http://docs.leggedrobotics.com/parameter_handler_doc/)

The software has been tested under ROS Melodic and Ubuntu 18.04.

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author(s):** Christian Gehring, Gabriel Hottiger, C. Dario Bellicoso

### Differences to ROS dynamic_reconfigure

[ROS dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure) can read and write parameters of nodes at run-time.
The tool has the following drawbacks compared to the parameter_handler packages:

* The client is not dynamically reconnecting when the server is restarted
* It requires a lot of overhead code.
* It does not support Eigen and kindr types.

## Building

In order to install, clone the latest version from this repository into your catkin workspace and compile the packages.

### Dependencies
* **[kindr](https://github.com/anybotics/kindr):** Kinematics and Dynamics for Robotics
* **[kindr_ros](https://github.com/anybotics/kindr_ros):** kindr ROS wrapper
* **[message_logger](https://github.com/anybotics/message_logger):** Message logging utility
* **Eigen3:** Linear alegra library
* **gtest:** Google's unit testing framework

## Usage

Please report bugs and request features using the [Issue Tracker](https://github.com/anybotics/parameter_handler/issues).

### Packages
* **parameter_handler:** Interface to parameter handler
* **parameter_handler_doc:** Doxygen documentation
* **parameter_handler_msgs:** ROS messages used by parameter_handler_ros
* **parameter_handler_ros:** An implementation of the parameter handler that uses ROS
* **parameter_handler_std:** An implementation of the parameter handler that is independent of ROS
* **rqt_parameters:** RQT plugin that provides a GUI for the parameter_handler_ros
